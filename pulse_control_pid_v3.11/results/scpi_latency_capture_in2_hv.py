#!/usr/bin/env python3
"""Measure 1 MHz trigger->photodiode latency with Red Pitaya SCPI.

Wiring:
  1 MHz trigger -> DIO0_P (EXT_PE)
  same trigger  -> attenuator/divider -> IN2
  photodiode    -> IN1

Requires the standard Red Pitaya acquisition FPGA/SCPI core.
"""
import argparse, csv, socket, time
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

ADC_FS_HZ = 125_000_000.0
DEFAULT_PULSE_HZ = 1_000_000.0

class SCPI:
    def __init__(self, host, port=5000, timeout=10):
        self.s = socket.create_connection((host, port), timeout=timeout)
        self.s.settimeout(timeout)
    def tx(self, cmd):
        self.s.sendall((cmd + "\r\n").encode())
    def query(self, cmd):
        self.tx(cmd); chunks=[]
        while True:
            b=self.s.recv(65536)
            if not b: raise ConnectionError("SCPI connection closed")
            chunks.append(b)
            d=b"".join(chunks)
            if d.endswith(b"\r\n") or d.endswith(b"\n"):
                return d.decode(errors="replace").strip()
    def array(self, cmd):
        t=self.query(cmd).strip().strip("{}")
        return np.fromstring(t, sep=",", dtype=float)
    def close(self):
        self.s.close()

def threshold(y):
    return float((np.percentile(y,10)+np.percentile(y,90))/2)

def edges(y, thr, polarity="positive"):
    a=y>=thr
    if polarity=="positive":
        return np.flatnonzero((~a[:-1]) & a[1:]) + 1
    return np.flatnonzero(a[:-1] & (~a[1:])) + 1

def acquire(rp, prewait_s, timeout_s):
    rp.tx("ACQ:START")
    time.sleep(prewait_s)
    rp.tx("ACQ:TRig EXT_PE")
    deadline=time.monotonic()+timeout_s
    while time.monotonic()<deadline:
        if rp.query("ACQ:TRig:FILL?").startswith("1"):
            return rp.array("ACQ:SOUR1:DATA?"), rp.array("ACQ:SOUR2:DATA?")
        time.sleep(0.001)
    raise TimeoutError("ACQ:TRig:FILL? timeout")

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--captures", type=int, default=100)
    ap.add_argument("--pulse-hz", type=float, default=DEFAULT_PULSE_HZ)
    ap.add_argument("--dec", type=int, default=1)
    ap.add_argument("--gain", choices=["LV","HV"], default="LV")
    ap.add_argument("--trigger-threshold-v", type=float)
    ap.add_argument("--pd-threshold-v", type=float)
    ap.add_argument("--pd-polarity", choices=["positive","negative"], default="positive")
    ap.add_argument("--min-latency-ns", type=float, default=50)
    ap.add_argument("--max-latency-ns", type=float, default=950)
    ap.add_argument("--pretrigger-wait-ms", type=float, default=1.0)
    ap.add_argument("--timeout-s", type=float, default=5.0)
    ap.add_argument("--prefix", default="latency")
    args=ap.parse_args()

    fs=ADC_FS_HZ/args.dec
    dt_ns=1e9/fs
    period_ns=1e9/args.pulse_hz
    period_samples=fs/args.pulse_hz
    print(f"pulse={args.pulse_hz/1e6:.6f} MHz, period={period_ns:.1f} ns")
    print(f"ADC={fs/1e6:.3f} MS/s, dt={dt_ns:.1f} ns, samples/period={period_samples:.1f}")
    if args.max_latency_ns >= period_ns:
        print("WARNING: latency search window >= one pulse period; matching can be ambiguous.")

    rp=SCPI(args.ip, timeout=args.timeout_s)
    rows=[]; raw1=[]; raw2=[]; first=None
    try:
        rp.tx("ACQ:RST")
        rp.tx(f"ACQ:DEC:Factor {args.dec}")
        rp.tx(f"ACQ:SOUR1:GAIN {args.gain}")
        rp.tx(f"ACQ:SOUR2:GAIN {args.gain}")
        rp.tx("ACQ:DATA:FORMAT ASCII")
        rp.tx("ACQ:DATA:Units VOLTS")
        rp.tx("ACQ:TRig:DLY 0")

        for k in range(args.captures):
            try:
                pd,trig=acquire(rp,args.pretrigger_wait_ms*1e-3,args.timeout_s)
                raw1.append(pd.astype(np.float32)); raw2.append(trig.astype(np.float32))
                center=len(trig)//2
                tt=args.trigger_threshold_v if args.trigger_threshold_v is not None else threshold(trig)
                te=edges(trig,tt,"positive")
                if len(te)==0: raise ValueError("no IN2 trigger edge")
                ti=int(te[np.argmin(np.abs(te-center))])
                if abs(ti-center)>max(4,int(np.ceil(period_samples/2))):
                    raise ValueError("no IN2 trigger edge near buffer center")

                pt=args.pd_threshold_v if args.pd_threshold_v is not None else threshold(pd)
                pe=edges(pd,pt,args.pd_polarity)
                lo=ti+int(np.floor(args.min_latency_ns/dt_ns))
                hi=ti+int(np.ceil(args.max_latency_ns/dt_ns))
                cand=pe[(pe>=lo)&(pe<=hi)]
                if len(cand)==0: raise ValueError("no PD edge in latency window")
                pi=int(cand[0])
                lat=(pi-ti)*dt_ns
                rows.append([k,1,lat,ti,pi,tt,pt,"none"])
                if first is None: first=(pd.copy(),trig.copy(),ti,pi)
                print(f"[{k+1:03d}/{args.captures}] latency={lat:8.3f} ns")
            except Exception as e:
                rows.append([k,0,"","","","","",str(e)])
                print(f"[{k+1:03d}/{args.captures}] INVALID: {e}")
    finally:
        rp.close()

    prefix=Path(args.prefix)
    with open(str(prefix)+"_results.csv","w",newline="") as f:
        w=csv.writer(f); w.writerow(["capture","valid","latency_ns","trigger_index","pd_index","trigger_threshold_v","pd_threshold_v","reason"]); w.writerows(rows)
    if raw1:
        np.savez_compressed(str(prefix)+"_raw.npz",in1_pd=np.stack(raw1),in2_trigger=np.stack(raw2),fs_hz=fs,pulse_hz=args.pulse_hz)

    vals=np.array([float(r[2]) for r in rows if r[1]==1])
    if len(vals)==0:
        print("No valid captures. Check wiring/thresholds."); return
    mean=float(np.mean(vals)); std=float(np.std(vals,ddof=1)) if len(vals)>1 else 0.0
    print(f"\nVALID {len(vals)}/{args.captures}  mean={mean:.3f} ns  std={std:.3f} ns  ADC step={dt_ns:.1f} ns")

    if first is not None:
        pd,trig,ti,pi=first
        half=int(2e-6*fs); a=max(0,ti-half); b=min(len(pd),ti+half+1)
        t=(np.arange(a,b)-ti)*dt_ns
        plt.figure(figsize=(10,5)); plt.plot(t,trig[a:b],label="IN2 physical trigger"); plt.plot(t,pd[a:b],label="IN1 photodiode")
        plt.axvline(0,ls="--"); plt.axvline((pi-ti)*dt_ns,ls=":",label=f"PD edge {(pi-ti)*dt_ns:.1f} ns")
        plt.xlabel("Time relative to trigger (ns)"); plt.ylabel("Voltage (V)"); plt.title("1 MHz trigger -> PD latency"); plt.grid(); plt.legend(); plt.tight_layout(); plt.savefig(str(prefix)+"_waveform.png",dpi=180); plt.close()

    plt.figure(figsize=(8,5)); plt.hist(vals,bins=min(30,max(5,int(np.sqrt(len(vals)))))); plt.axvline(mean,ls="--",label=f"mean={mean:.1f} ns")
    plt.xlabel("Latency (ns)"); plt.ylabel("Count"); plt.title("Trigger -> PD latency histogram"); plt.grid(); plt.legend(); plt.tight_layout(); plt.savefig(str(prefix)+"_histogram.png",dpi=180); plt.close()

    good=[r for r in rows if r[1]==1]; x=[r[0] for r in good]
    plt.figure(figsize=(9,5)); plt.plot(x,vals,"o-"); plt.axhline(mean,ls="--",label=f"mean={mean:.1f} ns")
    plt.xlabel("Capture index"); plt.ylabel("Latency (ns)"); plt.title("Latency stability"); plt.grid(); plt.legend(); plt.tight_layout(); plt.savefig(str(prefix)+"_vs_capture.png",dpi=180); plt.close()
    print("Saved *_results.csv, *_raw.npz, *_waveform.png, *_histogram.png, *_vs_capture.png")

if __name__=="__main__":
    main()
