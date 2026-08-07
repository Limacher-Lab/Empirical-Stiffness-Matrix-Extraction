import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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

if __name__ == "__main__":
    # 1. Define your sample nodes (nx2 vector)
    nodes = np.array([
        [18.342, -30.848],
        [18.342, -6.423],
        [18.342, 26.882]
    ])
    
    # 2. Define the averaging radius
    averaging_radius_mm = 1
    
    # 3. Process the file
    df = import_image_csv(r"DIC Data\Script Testing\Vz_03\B0012.csv")
    z_disp_vector = nodal_displacements(nodes, df, averaging_radius_mm)
    print(z_disp_vector)

    show_nodes_on_png(nodes, r"DIC Data\Script Testing\Vz_03\B0144.png", df, [74, 1275, 62, 806])