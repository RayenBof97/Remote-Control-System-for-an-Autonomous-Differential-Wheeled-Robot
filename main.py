import multiprocessing
import subprocess

def run_script(script_name):
    subprocess.call(["python", script_name])

if __name__ == "__main__":
    control_panel_process = multiprocessing.Process(target=run_script, args=("controlPanel.py",))
    plot_process = multiprocessing.Process(target=run_script, args=("plot.py",))

    control_panel_process.start()
    plot_process.start()

    control_panel_process.join()
    plot_process.join()

    print("Both scripts have finished executing.")
