import argparse
import json
import matplotlib.pyplot as plt
from pathlib import Path


def plot(plot_name: str, files: list[str], axis: str):
    log_dir = Path("logs")
    for file in files:
        with log_dir.joinpath(file).open("r") as f:
            flight_log = json.load(f)
            times = flight_log["times"]
            coordinates = flight_log[axis]
        plt.plot(times, coordinates, label=file[:-5], zorder=1)

    plt.xlabel("time")
    plt.ylabel(f"{axis}")

    plt.legend(loc="best")
    plt.savefig(f"{plot_name}.png", dpi=300)
    plt.show()


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--plot_name",
        type=str,
        help="name of the plot file",
    )
    parser.add_argument(
        "--files",
        nargs="+",
        type=str,
        help="list of files containing logging data",
    )
    parser.add_argument(
        "--axis",
        type=str,
        help="axis to be plotted",
    )

    args = parser.parse_args()
    plot(args.plot_name, args.files, args.axis)


if __name__ == "__main__":
    main()