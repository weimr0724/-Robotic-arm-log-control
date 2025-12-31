import os
import glob
import pandas as pd
import matplotlib.pyplot as plt


LOG_DIR = "logs"
OUT_DIR = "validation_out"


def find_latest_run_csv(log_dir: str = LOG_DIR) -> str | None:
    """Return the newest run_*.csv file path under log_dir, or None."""
    pattern = os.path.join(log_dir, "run_*.csv")
    files = glob.glob(pattern)
    if not files:
        return None
    return max(files, key=os.path.getmtime)


def ensure_out_dir(path: str = OUT_DIR):
    os.makedirs(path, exist_ok=True)


def save_target_vs_actual(df: pd.DataFrame, out_dir: str):
    """
    Save Target vs Actual plots for each joint.
    Expects columns like:
      target_a1, actual_a1, target_a2, actual_a2, target_a3, actual_a3
    """
    joints = [("a1", "Joint 1"), ("a2", "Joint 2"), ("a3", "Joint 3")]

    for j, title in joints:
        t_col = f"target_{j}"
        a_col = f"actual_{j}"
        if t_col in df.columns and a_col in df.columns:
            plt.figure()
            plt.plot(df[t_col], label="Target")
            plt.plot(df[a_col], label="Actual")
            plt.title(f"Target vs Actual ({title})")
            plt.xlabel("Sample")
            plt.ylabel("Angle (deg)")
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, f"target_vs_actual_{j}.png"), dpi=200)
            plt.close()


def save_error_over_time(df: pd.DataFrame, out_dir: str):
    """
    Save tracking error plots for each joint.
    If err_a1/err_a2/err_a3 exist, use them.
    Otherwise compute err = actual - target when possible.
    """
    joints = ["a1", "a2", "a3"]

    for j in joints:
        err_col = f"err_{j}"
        t_col = f"target_{j}"
        a_col = f"actual_{j}"

        series = None
        if err_col in df.columns:
            series = df[err_col]
        elif t_col in df.columns and a_col in df.columns:
            series = df[a_col] - df[t_col]

        if series is not None:
            plt.figure()
            plt.plot(series, label="Error")
            plt.title(f"Tracking Error vs Time (Joint {j.upper()})")
            plt.xlabel("Sample")
            plt.ylabel("Error (deg)")
            plt.legend()
            plt.tight_layout()
            plt.savefig(os.path.join(out_dir, f"error_{j}.png"), dpi=200)
            plt.close()


def print_summary(df: pd.DataFrame):
    """
    Print simple numeric summary:
    - mean absolute error
    - max absolute error
    for each joint when data is available.
    """
    joints = ["a1", "a2", "a3"]

    print("\n=== Validation Summary ===")
    for j in joints:
        err_col = f"err_{j}"
        t_col = f"target_{j}"
        a_col = f"actual_{j}"

        if err_col in df.columns:
            err = df[err_col]
        elif t_col in df.columns and a_col in df.columns:
            err = df[a_col] - df[t_col]
        else:
            print(f"- {j.upper()}: (no data columns found)")
            continue

        abs_err = err.abs()
        mae = abs_err.mean()
        mx = abs_err.max()
        print(f"- {j.upper()}: MAE={mae:.3f} deg, MaxAbs={mx:.3f} deg")


def main():
    latest = find_latest_run_csv(LOG_DIR)
    if latest is None:
        print(f"[ERROR] No run_*.csv found under: {LOG_DIR}")
        print("Run the control program to generate logs first.")
        return

    ensure_out_dir(OUT_DIR)
    print(f"[OK] Using latest log: {latest}")

    df = pd.read_csv(latest)

    # If your log includes a timestamp column, you can later switch x-axis to time.
    save_target_vs_actual(df, OUT_DIR)
    save_error_over_time(df, OUT_DIR)
    print_summary(df)

    print(f"\n[OK] Plots saved to: {OUT_DIR}")


if __name__ == "__main__":
    main()
