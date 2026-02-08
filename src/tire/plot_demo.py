import pandas as pd
import matplotlib.pyplot as plt
import os
import re  # For cleaning tire names
import argparse  # Import the argparse module

# Ensure plots directory exists
output_dir = "plots_by_tire"  # Directory where the generated plots will be saved
os.makedirs(output_dir, exist_ok=True)

# Define the normal forces for sampling (must match C++ code)
# This list is used to correctly identify columns in the CSVs
normal_forces_to_sample = [402, 802, 1202, 1602, 2001]

# Raw tire names (must match the names used in C++ code for consistency)
raw_tire_names = [
    "Bridgestone P255/35R18",
    "Bridgestone P225/40R18",
    "Continental P265/70R17",
    "Goodyear P225/60R16",
]


def clean_tire_name_for_filename(name):
    """
    Function to replace spaces and slashes in tire names to create valid filenames.
    This must match the `cleanTireName` function logic in the C++ code.
    """
    cleaned_name = name.replace(" ", "_").replace("/", "_")
    return cleaned_name


def plot_single_tire_csv_line(
    csv_directory,
    filename_suffix,
    x_col,
    y_col_prefix,
    xlabel,
    ylabel,
    title_base,
    current_tire_name_raw,
):
    """
    Plots data for a single tire, where different FZ values are represented by separate columns (lines).
    Generates one plot per tire type, showing multiple lines for different FZ conditions.
    This handles plot types 1, 2, 4, 6, and 8.
    """
    current_tire_name_cleaned = clean_tire_name_for_filename(current_tire_name_raw)
    csv_filename = f"{current_tire_name_cleaned}_{filename_suffix}.csv"
    full_csv_path = os.path.join(csv_directory, csv_filename)

    try:
        df = pd.read_csv(full_csv_path)
    except FileNotFoundError:
        print(
            f"Error: CSV file '{full_csv_path}' not found. Skipping plot for {current_tire_name_raw} - {title_base}."
        )
        return

    plt.figure(figsize=(12, 7))  # Increased figure size for better legend visibility

    # Dynamically find Y columns based on the prefix and sampled FZ values
    # Handles cases where alpha_deg is also part of the column name (e.g., FY_FZ_402_alpha_2deg)
    y_cols_to_plot = [
        col for col in df.columns if col.startswith(f"{y_col_prefix}_FZ_")
    ]

    if not y_cols_to_plot:
        print(
            f"No valid Y-axis columns found for {current_tire_name_raw} - {title_base}. Skipping plot."
        )
        plt.close()
        return

    for y_col in y_cols_to_plot:
        # Extract FZ value and optionally alpha_deg from column name for a descriptive legend
        fz_match = re.search(r"_FZ_(\d+)", y_col)
        fz_label = fz_match.group(1) if fz_match else "N/A"

        alpha_match = re.search(r"_alpha_(\d+)", y_col)
        alpha_label = f", α={alpha_match.group(1)}°" if alpha_match else ""

        plt.plot(df[x_col], df[y_col], label=f"FZ={fz_label} lbs{alpha_label}")

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f"{title_base} for {current_tire_name_raw}")
    plt.grid(True)
    # Place legend outside the plot to avoid overlapping with data
    plt.legend(
        title="Load Conditions",
        bbox_to_anchor=(1.02, 1),
        loc="upper left",
        borderaxespad=0.0,
    )
    plt.tight_layout(rect=[0, 0, 0.85, 1])  # Adjust layout to make room for the legend

    plot_filename = os.path.join(
        output_dir, f"{current_tire_name_cleaned}_{filename_suffix}.png"
    )
    plt.savefig(plot_filename)
    print(f"Generated plot: {plot_filename}")
    plt.close()


def plot_single_tire_csv_scatter(
    csv_directory,
    filename_suffix,
    x_col,
    y_col,
    color_col,
    xlabel,
    ylabel,
    title_base,
    current_tire_name_raw,
):
    """
    Plots data for a single tire as a scatter plot, coloring points by a specified column (e.g., Normal_Load_lbs).
    Generates one scatter plot per tire type.
    This handles plot types 3, 5, and 9.
    """
    current_tire_name_cleaned = clean_tire_name_for_filename(current_tire_name_raw)
    csv_filename = f"{current_tire_name_cleaned}_{filename_suffix}.csv"
    full_csv_path = os.path.join(csv_directory, csv_filename)

    try:
        df = pd.read_csv(full_csv_path)
    except FileNotFoundError:
        print(
            f"Error: CSV file '{full_csv_path}' not found. Skipping plot for {current_tire_name_raw} - {title_base}."
        )
        return

    plt.figure(figsize=(10, 6))

    # Use scatter plot with color mapping for the specified color_col (e.g., Normal_Load_lbs)
    scatter = plt.scatter(
        df[x_col],
        df[y_col],
        c=df[color_col],
        cmap="viridis",
        s=30,
        alpha=0.8,
        edgecolor="w",
        linewidth=0.5,
    )
    cbar = plt.colorbar(scatter)
    cbar.set_label(color_col.replace("_", " "))  # Label the color bar clearly

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f"{title_base} for {current_tire_name_raw}")
    plt.grid(True)
    plt.tight_layout()

    plot_filename = os.path.join(
        output_dir, f"{current_tire_name_cleaned}_{filename_suffix}.png"
    )
    plt.savefig(plot_filename)
    print(f"Generated plot: {plot_filename}")
    plt.close()


def plot_single_tire_csv_single_series(
    csv_directory,
    filename_suffix,
    x_col,
    y_col,
    xlabel,
    ylabel,
    title_base,
    current_tire_name_raw,
):
    """
    Plots data for a single tire where there's one X and one Y series.
    This is typically for plots where the X-axis is Normal Load and Y-axis is a derived property.
    Generates one plot per tire type.
    This handles plot types 7 and 10.
    """
    current_tire_name_cleaned = clean_tire_name_for_filename(current_tire_name_raw)
    csv_filename = f"{current_tire_name_cleaned}_{filename_suffix}.csv"
    full_csv_path = os.path.join(csv_directory, csv_filename)

    try:
        df = pd.read_csv(full_csv_path)
    except FileNotFoundError:
        print(
            f"Error: CSV file '{full_csv_path}' not found. Skipping plot for {current_tire_name_raw} - {title_base}."
        )
        return

    plt.figure(figsize=(10, 6))
    plt.plot(
        df[x_col], df[y_col], marker="o", linestyle="-", markersize=5
    )  # Added markers for clarity

    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.title(f"{title_base} for {current_tire_name_raw}")
    plt.grid(True)
    plt.tight_layout()

    plot_filename = os.path.join(
        output_dir, f"{current_tire_name_cleaned}_{filename_suffix}.png"
    )
    plt.savefig(plot_filename)
    print(f"Generated plot: {plot_filename}")
    plt.close()


def main():
    print("Starting generation of plots from CSV files (by tire type and load)...")

    # Setup argparse
    parser = argparse.ArgumentParser(
        description="Generate plots from tire model CSV data."
    )
    parser.add_argument(
        "csv_directory",
        type=str,
        help="The path to the directory where your CSV files are located.",
    )
    args = parser.parse_args()

    csv_directory = args.csv_directory
    if not os.path.isdir(csv_directory):
        print(
            f"Error: Directory '{csv_directory}' not found. Please provide a valid path."
        )
        return  # Exit if directory is invalid

    # Define the configurations for all plots
    plot_configurations = [
        {
            "type": "line",
            "suffix": "lateral_force_vs_slip_angle",
            "x_col": "Slip_Angle_deg",
            "y_prefix": "FY",
            "xlabel": "Slip Angle (degrees)",
            "ylabel": "Lateral Force (lbs)",
            "title_base": "Lateral Force vs Slip Angle",
        },
        {
            "type": "line",
            "suffix": "aligning_moment_vs_slip_angle",
            "x_col": "Slip_Angle_deg",
            "y_prefix": "MZ",
            "xlabel": "Slip Angle (degrees)",
            "ylabel": "Aligning Moment (ft-lbs)",
            "title_base": "Aligning Moment vs Slip Angle",
        },
        {
            "type": "scatter",
            "suffix": "aligning_moment_vs_lateral_force",
            "x_col": "Lateral_Force_lbs",
            "y_col": "Aligning_Moment_ft-lbs",
            "color_col": "Normal_Load_lbs",
            "xlabel": "Lateral Force (lbs)",
            "ylabel": "Aligning Moment (ft-lbs)",
            "title_base": "Aligning Moment vs Lateral Force",
        },
        {
            "type": "line",
            "suffix": "longitudinal_force_vs_slip",
            "x_col": "Longitudinal_Slip_pct",
            "y_prefix": "FX",
            "xlabel": "Longitudinal Slip (%)",
            "ylabel": "Longitudinal Force (lbs)",
            "title_base": "Longitudinal Force vs Longitudinal Slip",
        },
        {
            "type": "scatter",
            "suffix": "friction_circle",
            "x_col": "Longitudinal_Force_lbs",
            "y_col": "Lateral_Force_lbs",
            "color_col": "Normal_Load_lbs",
            "xlabel": "Longitudinal Force (lbs)",
            "ylabel": "Lateral Force (lbs)",
            "title_base": "Lateral Force vs Longitudinal Force (Friction Circle)",
        },
        {
            "type": "line",
            "suffix": "lateral_force_vs_longitudinal_slip",
            "x_col": "Longitudinal_Slip_pct",
            "y_prefix": "FY",
            "xlabel": "Longitudinal Slip (%)",
            "ylabel": "Lateral Force (lbs)",
            "title_base": "Lateral Force vs Longitudinal Slip (Combined Slip)",
        },
        {
            "type": "single_series",
            "suffix": "longitudinal_peak_friction_vs_load",
            "x_col": "Normal_Load_lbs",
            "y_col": "MUXp",
            "xlabel": "Normal Load (lbs)",
            "ylabel": "Longitudinal Peak Friction Coefficient",
            "title_base": "Longitudinal Peak Friction vs Normal Load",
        },
        {
            "type": "line",
            "suffix": "longitudinal_force_combined_slip",
            "x_col": "Longitudinal_Slip_pct",
            "y_prefix": "FX",
            "xlabel": "Longitudinal Slip (%)",
            "ylabel": "Longitudinal Force (lbs)",
            "title_base": "Longitudinal Force vs Slip (Combined Conditions)",
        },
        {
            "type": "scatter",
            "suffix": "lateral_vs_longitudinal_peak_friction",
            "x_col": "MUXp",
            "y_col": "MUYp",
            "color_col": "Normal_Load_lbs",
            "xlabel": "Longitudinal Peak Friction Coefficient",
            "ylabel": "Lateral Peak Friction Coefficient",
            "title_base": "Lateral vs Longitudinal Peak Friction",
        },
        {
            "type": "single_series",
            "suffix": "lateral_stiffness_vs_load",
            "x_col": "Normal_Load_lbs",
            "y_col": "C_alpha",
            "xlabel": "Normal Load (lbs)",
            "ylabel": "Lateral Stiffness C_alpha (lbs/deg)",
            "title_base": "Lateral Stiffness vs Normal Load",
        },
    ]

    # Loop through each tire type to generate its specific plots
    for raw_tire_name in raw_tire_names:
        print(f"\n--- Processing plots for {raw_tire_name} ---")

        for plot_config in plot_configurations:
            plot_type = plot_config["type"]

            # Common arguments
            common_args = {
                "csv_directory": csv_directory,  # Pass the selected directory
                "filename_suffix": plot_config["suffix"],
                "x_col": plot_config["x_col"],
                "xlabel": plot_config["xlabel"],
                "ylabel": plot_config["ylabel"],
                "title_base": plot_config["title_base"],
                "current_tire_name_raw": raw_tire_name,
            }

            if plot_type == "line":
                plot_single_tire_csv_line(
                    y_col_prefix=plot_config["y_prefix"], **common_args
                )
            elif plot_type == "scatter":
                plot_single_tire_csv_scatter(
                    y_col=plot_config["y_col"],
                    color_col=plot_config["color_col"],
                    **common_args,
                )
            elif plot_type == "single_series":
                plot_single_tire_csv_single_series(
                    y_col=plot_config["y_col"], **common_args
                )
            else:
                print(f"Unknown plot type: {plot_type}. Skipping.")

    print("\nAll plots generated and saved in the 'plots_by_tire' directory!")


if __name__ == "__main__":
    main()
