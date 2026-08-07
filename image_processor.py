import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob
from scipy.stats import linregress

def import_image_csv(filepath):

    df = pd.read_csv(filepath, sep=None, engine='python')
    
    # Strip whitespace from column names just in case
    df.columns = df.columns.str.strip()
    
    # Optional debugging line so you can see what Pandas actually found:
    # print(f"Detected columns: {list(df.columns)}") 
    
    return df

def nodal_displacements(nodes, df, radius):

    # Ensure nodes is a numpy array
    nodes = np.array(nodes)
    radius_sq = radius ** 2
    displacements = []

    # Map column names (Adjust these strings if your CSV headers differ exactly)
    # Using .values extracts the raw numpy arrays from pandas, 
    # which is significantly faster for the math we are about to do.
    col_x = 'x [mm]' if 'x [mm]' in df.columns else df.columns[0]
    col_y = 'y [mm]' if 'y [mm]' in df.columns else df.columns[1]
    col_z = 'z-displacement [mm]' if 'z-displacement [mm]' in df.columns else df.columns[2]

    df_x = df[col_x].values
    df_y = df[col_y].values
    df_z = df[col_z].values

    # Iterate through each given node
    for node in nodes:
        nx, ny = node[0], node[1]
        
        # Vectorized Euclidean distance squared calculation
        # We use distance squared to avoid the computationally expensive math.sqrt()
        dist_sq = (df_x - nx)**2 + (df_y - ny)**2
        
        # Create a boolean mask of all points that fall within the circle
        mask = dist_sq <= radius_sq
        
        # Extract the z-displacements for only those points
        points_in_radius = df_z[mask]
        
        # Calculate the area average
        if len(points_in_radius) > 0:
            avg_z = np.mean(points_in_radius)
        else:
            # If a node is placed where no DIC data exists, return NaN to prevent skewed data
            avg_z = np.nan 
            
        displacements.append(avg_z)

    # Return as an nx1 column vector to match your pipeline requirements
    return np.array(displacements).reshape(-1, 1)

def show_nodes_on_png(nodes, image_path, df, csv_bounds_px):

    # 1. Extract physical boundaries [mm] directly from the CSV dataframe
    # We assume X is column 0 and Y is column 1 based on your DIC output
    col_x, col_y = df.columns[0], df.columns[1]
    
    x_min_mm, x_max_mm = df[col_x].min(), df[col_x].max()
    y_min_mm, y_max_mm = df[col_y].min(), df[col_y].max()
    
    # 2. Extract the pixel boundaries from your input
    left_px, right_px, top_px, bottom_px = csv_bounds_px
    
    # 3. Read the PNG image
    img = plt.imread(image_path)
    
    # ==========================================
    # MAPPING ENGINE: [mm] -> [pixels]
    # ==========================================
    nodes = np.array(nodes)
    nodes_x_mm = nodes[:, 0]
    nodes_y_mm = nodes[:, 1]
    
    # Map X-axis: physical min/max maps to pixel left/right
    nodes_x_px = np.interp(nodes_x_mm, [x_min_mm, x_max_mm], [left_px, right_px])
    
    # Map Y-axis: physical min/max maps to pixel bottom/top
    # Note: In images, Y=0 is the TOP. So the physical minimum Y (bottom of sample) 
    # maps to the bottom_px (which is a larger pixel number than top_px).
    nodes_y_px = np.interp(nodes_y_mm, [y_min_mm, y_max_mm], [bottom_px, top_px])
    
    # Create the bounding box corners for visual validation
    box_x = [left_px, right_px, right_px, left_px, left_px]
    box_y = [bottom_px, bottom_px, top_px, top_px, bottom_px]

    # ==========================================
    # RENDER THE VISUALIZATION
    # ==========================================
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Display the raw image
    ax.imshow(img, cmap='gray')
    
    # Draw a dashed red box showing exactly where the script thinks the CSV data lives
    ax.plot(box_x, box_y, 'r--', linewidth=2, label='CSV Data Pixel Boundary')
    
    # Plot the Pokinator target nodes as crosshairs
    ax.scatter(nodes_x_px, nodes_y_px, c='red', s=70, marker='+', linewidths=1, label='Nodes')
    # Loop through the pixel coordinates using enumerate to get an index (i)
    for i, (x, y) in enumerate(zip(nodes_x_px, nodes_y_px)):
        ax.annotate(str(i + 1), 
                    (x, y), 
                    textcoords="offset points", 
                    xytext=(4, -8), 
                    ha='left', 
                    color='red', 
                    fontsize=10, 
                    fontweight='bold')

    # Formatting
    ax.set_title('Node Locations')
    ax.set_xlabel('X [pixels]')
    ax.set_ylabel('Y [pixels]')
    #ax.legend(loc='upper left')
    ax.grid(False) 
    
    plt.show()

def compare_beam_theory(weight_g, load_loc_mm, exp_nodes_mm, exp_disp_mm, b_mm=20, h_mm=0.970, L_mm=200, E_GPa=64.18):
    """
    Compares experimental DIC nodal displacements with a 1D Euler-Bernoulli 
    cantilever beam model and plots the results. Supports multiple point loads
    via superposition.
    
    Parameters:
    - b_mm : Beam width (mm)
    - h_mm : Beam thickness (mm)
    - L_mm : Beam total length (mm)
    - E_GPa : Modulus of Elasticity (GPa)
    - weight_g : Applied load mass (grams) - float or array-like
    - load_loc_mm : Distance from the clamped base to the applied load (mm) - float or array-like
    - exp_nodes_mm : Array/List of nodal x-coordinates from DIC (mm)
    - exp_disp_mm : Array/List of nodal z-displacements from DIC (mm)
    """
    
    # 0. Normalize inputs to numpy arrays (handles both floats and lists gracefully)
    weights = np.atleast_1d(weight_g)
    locs = np.atleast_1d(load_loc_mm)
    
    # Pre-flight check to ensure the loads and locations match
    if len(weights) != len(locs):
        raise ValueError("[!] Error: The number of provided weights must match the number of load locations.")
    
    # 1. Unit Conversions for standard [N, mm, MPa] framework
    E_MPa = E_GPa * 1000.0                # 1 GPa = 1000 MPa (N/mm^2)
    
    # 2. Area Moment of Inertia (I) for load normal to width
    I = (b_mm * (h_mm**3)) / 12.0         # mm^4
    
    # 3. Generate theoretical high-resolution X array
    x_theory = np.linspace(0, L_mm, 500)
    
    # 4. Calculate Euler-Bernoulli deflection (Superposition)
    # Initialize a zero-array to aggregate the deflections
    v_theory_total = np.zeros_like(x_theory)
    
    for F_mass, a_loc in zip(weights, locs):
        # Convert this specific weight to Newtons
        F_N = (F_mass * 9.81) / 1000.0    
        
        # Calculate deflection for this specific load
        # Equation 1: 0 <= x <= a (between fixed base and load)
        # Equation 2: a < x <= L (between load and free end)
        v_theory_current = np.where(
            x_theory <= a_loc,
            (F_N * x_theory**2) / (6 * E_MPa * I) * (3 * a_loc - x_theory),
            (F_N * a_loc**2) / (6 * E_MPa * I) * (3 * x_theory - a_loc)
        )
        
        # Add to the total beam deflection (Linear Superposition)
        v_theory_total += v_theory_current
    
    # NOTE: Euler-Bernoulli yields a positive magnitude for deflection. 
    # If your DIC outputs negative Z values for a downward displacement, 
    # you may need to invert the sign of v_theory or take the absolute of exp_disp_mm.
    # For now, we plot the absolute magnitude.
    exp_disp_abs = np.abs(exp_disp_mm)
    
    # 5. Plotting the Comparison
    plt.figure(figsize=(10, 6))
    
    # Theoretical curve
    plt.plot(x_theory, v_theory_total, label='Euler-Bernoulli Theory (Superposition)', color='#1f77b4', linewidth=2.5, zorder=1)
    
    # Experimental nodal data
    plt.scatter(exp_nodes_mm, exp_disp_abs, label='Pokinator/DIC Experimental', 
                color='#ff7f0e', marker='x', s=60, linewidths=2, zorder=2)
    
    # Annotate load position(s)
    for i, a_loc in enumerate(locs):
        # Only add the label to the legend once to keep it clean
        label_str = 'Load Application Point' if i == 0 else ""
        plt.axvline(x=a_loc, color='gray', linestyle='--', alpha=0.5, label=label_str)
    
    # Dynamic Formatting
    if len(weights) == 1:
        subtitle = f"(Load: {weights[0]}g at {locs[0]}mm)"
    else:
        subtitle = f"(Multiple Loads: {len(weights)} Points of Contact)"
        
    plt.title(f'Beam Deflection Profile: Experimental vs. Theory\n{subtitle}', fontsize=14, fontweight='bold')
    plt.xlabel('Distance from Clamped Base, X (mm)', fontsize=12)
    plt.ylabel('Absolute Deflection, Z (mm)', fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend(fontsize=11)
    plt.tight_layout()
    
    plt.show()

def single_point_euler_beam():
    # 1. Define your sample nodes (nx2 vector)
    #nodes = np.array([[60, -97]])
    #nodes = np.array([[60.0, -97.0 + (i * 8)] for i in range(20)]) #nodes for 200g-100mm
    #nodes = np.array([[38.0, -119.0 + (i * 10)] for i in range(20)]) #nodes for 300g-50mm
    nodes = np.array([[0, -(i * 10)] for i in range(20)]) #nodes for 3-point set
    #print(nodes[:,1]+97)

    # 2. Define the averaging radius
    averaging_radius_mm = 1
    
    # 3. Process the file
    df = import_image_csv(r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\3-point-set\B0002.csv")
    z_disp_vector = nodal_displacements(nodes, df, averaging_radius_mm)
    #print(z_disp_vector)

    #show_nodes_on_png(nodes, r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\300g-50mm\B0002-jpg.jpg", df, [74, 1275, 43, 824])
    # compare_beam_theory(20, 0.970, 200, 68.9, 110.10, 150, abs(nodes[:,1]), z_disp_vector)
    # compare_beam_theory(20, 0.970, 200, 64.18, 110.10, 150, abs(nodes[:,1]), z_disp_vector) # 3-point-set load 3 
    # compare_beam_theory(20, 0.970, 200, 64.18, 187.73, 100, abs(nodes[:,1]), z_disp_vector) # 3-point-set load 2
    compare_beam_theory(20, 0.970, 200, 64.18, 262.64, 50, abs(nodes[:,1]), z_disp_vector) # 3-point-set load 1

def single_element_beam_k(node_loc, image_paths, force_csv_path, width, thickness, length, averaging_radius=2.0):
    """
    Computes beam stiffness (k) and Young's Modulus (E) from experimental DIC and Force data.
    Assumes load cell forces are in grams and converts them natively to Newtons.
    """
    
    # 1. Process Force Data
    force_df = pd.read_csv(force_csv_path)
    
    # Calculate the average time step (dt) to find how many rows make up 5 seconds
    avg_dt = force_df['Time'].diff().mean()
    entries_in_5s = int(round(5.0 / avg_dt))
    
    # Extract only the force columns, preserving their order
    force_cols = [col for col in force_df.columns if 'Force' in col]
    mean_forces_N = []
    
    for col in force_cols:
        # Drop any NaNs to find the actual end of recording for this specific node
        valid_data = force_df[col].dropna()
        
        # Extract the last 5 seconds worth of entries
        last_5s_data = valid_data.iloc[-entries_in_5s:]
        
        # Average the steady-state data and convert from grams to Newtons
        mean_force_grams = last_5s_data.mean()
        # Force the result to be a strict scalar float
        mean_force_N = float((mean_force_grams * 9.81) / 1000.0) 
        mean_forces_N.append(mean_force_N)

    # 2. Process DIC Data
    # node_loc must be formatted as an nx2 array for nodal_displacements
    node_array = np.array([node_loc])
    displacements = []
    
    for path in image_paths:
        # Utilize the previously built custom CSV importer
        dic_df = import_image_csv(path)
        
        # Calculate displacement and cast to absolute magnitude for a positive plot 
        disp_vector = nodal_displacements(node_array, dic_df, averaging_radius)
        
        # Squeeze out nested array dimensions and force it to be a Python float
        abs_disp = float(np.abs(np.squeeze(disp_vector))) 
        displacements.append(abs_disp)

    # Match force data to displacement data and strictly flatten to 1D arrays
    n_points = min(len(mean_forces_N), len(displacements))
    x_data = np.array(displacements[:n_points]).flatten()
    y_data = np.array(mean_forces_N[:n_points]).flatten()

    # 3. Compute Stiffness via Linear Regression
    slope, intercept, r_value, p_value, std_err = linregress(x_data, y_data)
    r_squared = r_value**2
    k_stiffness = slope # Units: N/mm

    # 4. Compute Young's Modulus (E)
    # Area Moment of Inertia for load applied normal to width
    I = (width * (thickness**3)) / 12.0
    
    # Euler-Bernoulli equation for end-loaded cantilever: k = (3 * E * I) / L^3
    # Rearranged for E: E = (k * L^3) / (3 * I)
    E_MPa = (k_stiffness * (length**3)) / (3.0 * I)
    E_GPa = E_MPa / 1000.0

    # 5. Build the Visualization
    plt.figure(figsize=(9, 6))
    
    # Scatter plot of physical data
    plt.scatter(x_data, y_data, color='cyan', edgecolor='black', s=80, zorder=4, label='Experimental Nodes')

    # Generate extended x values to force the line of best fit to start at x=0
    x_fit = np.array([0.0, max(x_data)])
    fit_line = slope * x_fit + intercept
    
    # Plot the extended line of best fit
    plt.plot(x_fit, fit_line, color='magenta', linestyle='--', linewidth=2, zorder=3, label='Linear Fit')

    # Formatting
    plt.title("Experimental Force vs. Deflection", fontsize=14, fontweight='bold')
    plt.xlabel("Deflection (mm)", fontsize=12)
    plt.ylabel("Force (N)", fontsize=12)
    plt.grid(True, linestyle=':', alpha=0.7)

    # Explicitly draw origin lines and lock the X-axis to 0
    plt.axhline(0, color='black', linewidth=1.5, zorder=2, alpha=0.8) # X-axis
    plt.axvline(0, color='black', linewidth=1.5, zorder=2, alpha=0.8) # Y-axis
    plt.xlim(left=0)

    # Floating text box for the calculated physics values (now including y-intercept)
    text_str = (f"$R^2$: {r_squared:.4f}\n"
                f"y-int: {intercept:.2f} N\n"
                f"Stiffness ($k$): {k_stiffness:.2f} N/mm\n"
                f"Modulus ($E$): {E_GPa:.2f} GPa")
    
    plt.text(0.05, 0.95, text_str, transform=plt.gca().transAxes, fontsize=12,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='black', alpha=0.8, edgecolor='cyan'))
    
    # Set text color in the box to white to contrast with the black background
    plt.gca().texts[-1].set_color('white')

    plt.legend(loc='lower right')
    plt.tight_layout()
    plt.show()

    return k_stiffness, E_GPa

def extract_k_matrix(force_file_path, dic_folder_path, node_locations, averaging_radius_mm=1.0):
    """
    Constructs the Stiffness Matrix [K] from Pokinator force data and DIC displacements.
    Handles ragged CSV columns (varying lengths) by dropping NaNs per column.
    Assumes load cell forces are in grams and converts them natively to Newtons.
    """

    # STEP 1: Process Force Data ([F] Matrix Initialization)
    df_force = pd.read_csv(force_file_path)
    
    # Identify the time column
    time_col = next((col for col in df_force.columns if 'time' in col.lower()), None)
    if not time_col:
        raise ValueError("Could not locate a 'Time' column in the force CSV.")
        
    # Calculate the average time step (dt) to find how many rows make up 5 seconds
    avg_dt = df_force[time_col].diff().mean()
    entries_in_5s = int(round(5.0 / avg_dt))
    
    # Extract just the force columns (ignores time)
    force_cols = [col for col in df_force.columns if col != time_col]
    n_nodes = len(node_locations)
    
    # STEP 2: Sort and Validate DIC Data
    # Grab all 'B*.csv' files and sort them to guarantee B0001, B0002 order
    dic_search_path = os.path.join(dic_folder_path, 'B*.csv')
    dic_files = sorted(glob.glob(dic_search_path))
    
    # Validation checks to prevent misaligned matrices
    if len(force_cols) != len(dic_files):
        raise ValueError(f"Matrix Dimension Mismatch: {len(force_cols)} force columns vs {len(dic_files)} DIC files.")
    if len(force_cols) != n_nodes:
        raise ValueError(f"System Dimension Mismatch: Evaluated {len(force_cols)} forces/images, but {n_nodes} node locations were provided.")

    # STEP 3: Build the Force Matrix [F]
    F_matrix = np.zeros((n_nodes, n_nodes))
    
    for i, col in enumerate(force_cols):
        # Drop any NaNs to find the actual end of recording for this specific node
        valid_data = df_force[col].dropna()
        
        # Extract the last 5 seconds worth of entries
        last_5s_data = valid_data.iloc[-entries_in_5s:]
        
        # Average the steady-state data and convert from grams to Newtons
        mean_force_grams = last_5s_data.mean()
        mean_force_N = float((mean_force_grams * 9.81) / 1000.0) 
        
        # Place in the diagonal, making all other entries in the column 0
        F_matrix[i, i] = mean_force_N

    # STEP 4: Build the Displacement Matrix [X]
    X_matrix = np.zeros((n_nodes, n_nodes))
    
    for i, dic_path in enumerate(dic_files):
        # Utilize your existing DIC processing functions
        df_dic = import_image_csv(dic_path)
        
        # Calculate area-averaged Z-displacements for all nodes during this poke
        x_vector = nodal_displacements(node_locations, df_dic, averaging_radius_mm)
        
        # Append as a column right-on into the [X] matrix
        X_matrix[:, i] = x_vector.flatten()

    # STEP 5: Calculate the Stiffness Matrix [K]
    # Calculate [K] = [F] * [X]^-1
    try:
        X_inv = np.linalg.inv(X_matrix)
    except np.linalg.LinAlgError:
        # Failsafe: If experimental noise makes [X] slightly singular
        print("Warning: [X] matrix is singular. Falling back to Moore-Penrose pseudo-inverse.")
        X_inv = np.linalg.pinv(X_matrix)
        
    K_matrix = np.matmul(F_matrix, X_inv)
    
    return F_matrix, X_matrix, K_matrix

def print_matrix(matrix):
    # Convert the array to a DataFrame with Node labels
    n_nodes = len(matrix)
    node_labels = [f"Node {i+1}" for i in range(n_nodes)]

    df = pd.DataFrame(matrix, columns=node_labels, index=node_labels)

    # print("\n--- Experimental Stiffness Matrix [K] ---")
    print(df.round(2).to_string()) # to_string() prevents Pandas from hiding middle columns

def plot_matrix_heatmap(matrix, title="Matrix"):
    plt.figure(figsize=(8, 6))
    
    # Create the heatmap. 'viridis' or 'coolwarm' are excellent colormaps for this.
    plt.imshow(matrix, cmap='viridis', aspect='auto')
    
    # Add a color scale bar
    plt.colorbar(label='Magnitude')
    
    # Formatting
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel("Displacement Node [X]", fontsize=12)
    plt.ylabel("Force Node [F]", fontsize=12)
    
    # Force integer ticks for the nodes
    nodes = range(len(matrix))
    plt.xticks(nodes, [f"{i+1}" for i in nodes])
    plt.yticks(nodes, [f"{i+1}" for i in nodes])
    
    plt.tight_layout()
    plt.show()

def plot_norm_matrix_heatmap(matrix, title="Normalized Deflection Matrix"):
    """
    Plots a heatmap of a matrix. Normalizes the matrix column-by-column 
    so each column's maximum value (tip deflection) scales to 1.0.
    """
    # KINEMATIC MATH: Normalize column-by-column. 
    # np.max(matrix, axis=0) creates an array of the max values for EACH column independently.
    # NumPy automatically "broadcasts" this division down the columns.
    normalized_matrix = matrix / np.max(matrix, axis=0)
    print(round(np.linalg.cond(normalized_matrix)),2)
    
    # NOTE: If you actually wanted to normalize against the GLOBAL maximum 
    # of the entire dataset, you would replace the line above with:
    # normalized_matrix = matrix / np.max(matrix)

    plt.figure(figsize=(8, 6))
    
    # Create the heatmap. 'viridis' or 'coolwarm' are excellent colormaps for this.
    plt.imshow(normalized_matrix, cmap='viridis', aspect='auto')
    
    # Add a color scale bar. Label updated to reflect normalization!
    plt.colorbar(label='Normalized Magnitude (0.0 - 1.0)')
    
    # Formatting
    plt.title(title, fontsize=14, fontweight='bold')
    plt.xlabel("Displacement Node [X]", fontsize=12)
    plt.ylabel("Force Node [F]", fontsize=12)
    
    # Force integer ticks for the nodes
    nodes = range(len(matrix))
    plt.xticks(nodes, [f"{i+1}" for i in nodes])
    plt.yticks(nodes, [f"{i+1}" for i in nodes])
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # single_point_euler_beam()
    # single_element_beam_k(node_loc=[0, -100],
    #                       image_paths=[r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0002.csv",
    #                                    r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0003.csv",
    #                                    r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0004.csv",
    #                                    r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0005.csv",
    #                                    r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0006.csv",
    #                                    r"C:\Users\Haziq Sabri\pokinator_env\DIC Data\Single-Element-Beam-k-1\B0007.csv"],
    #                         force_csv_path=r"C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Single-Element_Beam-k-1.csv",
    #                         width=20, thickness=0.970, length=100, averaging_radius=1)

    zero_loc = -30
    nodes = np.array([[0, zero_loc-(i * 10)] for i in range(15)])
    F, X, K = extract_k_matrix(force_file_path=r'C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Euler-Beam-K-Extract-1.csv',
                     dic_folder_path=r'C:\Users\Haziq Sabri\pokinator_env\DIC Data\Euler-Beam-K-Extract-1',
                     node_locations=nodes)

    # nodes = np.array([[0, -80-(i * 10)] for i in range(10)])
    # F, X, K = extract_k_matrix(force_file_path=r'C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Euler-Beam-K-Extract-1_Reduced-1.csv',
    #                      dic_folder_path=r'C:\Users\Haziq Sabri\pokinator_env\DIC Data\Euler-Beam-K-Extract-1_Reduced-1',
    #                      node_locations=nodes)

    # nodes = np.array([[0, -110-(i * 10)] for i in range(7)])
    # F, X, K = extract_k_matrix(force_file_path=r'C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Euler-Beam-K-Extract-1_Reduced-2.csv',
    #                      dic_folder_path=r'C:\Users\Haziq Sabri\pokinator_env\DIC Data\Euler-Beam-K-Extract-1_Reduced-2',
    #                      node_locations=nodes)

    # nodes = np.array([[0, -30-(i * 20)] for i in range(8)])
    # F, X, K = extract_k_matrix(force_file_path=r'C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Euler-Beam-K-Extract-1_Reduced-3.csv',
    #                     dic_folder_path=r'C:\Users\Haziq Sabri\pokinator_env\DIC Data\Euler-Beam-K-Extract-1_Reduced-3',
    #                     node_locations=nodes)

    # nodes = np.array([[0, -30-(i * 40)] for i in range(4)])
    # F, X, K = extract_k_matrix(force_file_path=r'C:\Users\Haziq Sabri\pokinator_env\Rec. Forces\Euler Beam v2\Euler-Beam-K-Extract-1_Reduced-4.csv',
    #                     dic_folder_path=r'C:\Users\Haziq Sabri\pokinator_env\DIC Data\Euler-Beam-K-Extract-1_Reduced-4',
    #                     node_locations=nodes)

    # print_matrix(F)
    # plot_matrix_heatmap(X)
    # plot_norm_matrix_heatmap(X)
    # print(round(np.linalg.cond(X),2))
    # print(F)

    # node_shape = np.shape(nodes) 
    # force_vector = np.zeros((node_shape[0], 1))
    # load_g = 100
    # load_loc = [100, 170]
    # force_vector[7] = load_g * 9.81 / 1000
    # force_vector[14] = 4*load_g * 9.81 / 1000
    # deflection = np.linalg.inv(K) @ force_vector
    # compare_beam_theory(weight_g=[load_g,4*load_g], load_loc_mm=load_loc, exp_nodes_mm=abs(nodes[:,1]), exp_disp_mm=deflection)

    load_g = np.ones(15)*100
    load_loc = np.array([[30+i*10] for i in range(15)])
    force_vector = np.ones(15)*(100*9.81)/1000
    deflection = np.linalg.inv(K) @ force_vector
    compare_beam_theory(weight_g=load_g, load_loc_mm=load_loc, exp_nodes_mm=abs(nodes[:,1]), exp_disp_mm=deflection)