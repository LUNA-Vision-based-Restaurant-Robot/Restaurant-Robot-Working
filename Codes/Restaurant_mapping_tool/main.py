import tkinter as tk
from tkinter import messagebox
from tkinter import ttk
from PIL import Image, ImageTk

start_point = [20,20]
Restaurant_name = "Restaurant"

def save_avr_code_to_file(filename, code):
    with open(filename, 'w') as file:
        file.write(code)

def generate_avr_code(left, table_metadata):
    avr_code = """
#include <avr/eeprom.h>

#define LEFT_ADDR 0x00
#define METADATA_START_ADDR 0x02

void save_data_to_eeprom() {{
    // Save left variable to EEPROM
    eeprom_update_byte((uint8_t*)LEFT_ADDR, {left});

    // Save table_metadata to EEPROM
    uint16_t metadata_addr = METADATA_START_ADDR;
""".format(left=left)

    for index, (table_number, start_junction, count) in enumerate(table_metadata):
        avr_code += """
    eeprom_update_word((uint16_t*)(metadata_addr + {index}*6), {table_number});
    eeprom_update_word((uint16_t*)(metadata_addr + {index}*6 + 2), {start_junction});
    eeprom_update_word((uint16_t*)(metadata_addr + {index}*6 + 4), {count});
""".format(index=index, table_number=table_number, start_junction=start_junction, count=count)

    avr_code += """
}}

void retrieve_data_from_eeprom() {{
    // Retrieve left variable from EEPROM
    uint8_t left = eeprom_read_byte((const uint8_t*)LEFT_ADDR);

    // Retrieve table_metadata from EEPROM
    uint16_t metadata_addr = METADATA_START_ADDR;
    for (uint8_t i = 0; i < {metadata_len}; i++) {{
        uint16_t table_number = eeprom_read_word((const uint16_t*)(metadata_addr + i*6));
        uint16_t start_junction = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 2));
        int16_t count = eeprom_read_word((const uint16_t*)(metadata_addr + i*6 + 4));
        // Use the retrieved data as needed
    }}
}}

int main(void) {{
    save_data_to_eeprom();
    retrieve_data_from_eeprom();

    while (1) {{
        // Main loop
    }}
}}
""".format(metadata_len=len(table_metadata))

    return avr_code

def draw_tables(canvas, table_info_list):
    for table_info in table_info_list:
        table_number, table_center_x, table_center_y, table_width, table_length = table_info

        table_top_left_x = table_center_x - (table_width / 2)
        table_top_left_y = table_center_y - (table_length / 2)
        table_bottom_right_x = table_center_x + (table_width / 2)
        table_bottom_right_y = table_center_y + (table_length / 2)

        canvas.create_rectangle(table_top_left_x, table_top_left_y, table_bottom_right_x, table_bottom_right_y, fill="white")
        canvas.create_text(table_center_x, table_bottom_right_y + 10, text=f"Table {table_number}", anchor=tk.N)

def draw_shortest_path(canvas, table_info_list):
    global Restaurant_name
    global start_point

    #start line vertical comes 20px down
    canvas.create_line(start_point[0], start_point[1], start_point[0], start_point[1] + 20, fill="white", width=4)
    #draw start point
    canvas.create_oval(start_point[0] - 10, start_point[1] - 10, start_point[0] + 10, start_point[1] + 10, fill="red")

    # print(table_info_list)
    center_points_horizontal = []

    for table_info in table_info_list:
        center_points_horizontal.append([table_info[1], table_info[3]])

    #remove duplicates and sort based on the x value
    center_points_horizontal = sorted(list(set(map(tuple, center_points_horizontal))))

    print(center_points_horizontal)

    points_horizontal = []
    odd = True
    for center_point in center_points_horizontal:
        if odd:
            points_horizontal.append(center_point[0] + center_point[1]/2 + 20)
        else:
            points_horizontal.append(center_point[0] - center_point[1]/2 - 20)
        odd = not odd

    print(points_horizontal)

    #calculate vertical line end points finding what is the maximum distance to a table in a row
    points_vertical = []
    for i in center_points_horizontal:
        temp = 0
        temp_min = 100000
        for j in table_info_list:
            if j[1] == i[0]:
                if j[2] > temp:
                    temp = j[2]
                if j[2] < temp_min:
                    temp_min = j[2]
        points_vertical.append([temp, temp_min])

    print(points_vertical)

    #make a list of tables with the format (table_number, start junction, count(+/-))
    table_metadata = []
    for i in table_info_list:
        #find the start junction
        for j in range(len(center_points_horizontal)):
            if i[1] == center_points_horizontal[j][0]:
                start_junction = j

        #find the count
        #take a list of tables in the same row
        temp = []
        for j in table_info_list:
            if j[1] == i[1]:
                temp.append(j)

        #sort the list based on the y value
        temp = sorted(temp, key=lambda x: x[2])

        #start[1] + 20 is the starting point of the vertical line, split the table list based on top or buttom of the start line
        up = []
        for j in range(len(temp)):
            if temp[j][2] < start_point[1] + 20:
                up.append(temp[j])

        #split the temp list based on j position into 2 lists up and down
        down = []
        for j in range(len(temp)):
            if temp[j][2] > start_point[1] + 20:
                down.append(temp[j])

        #revserse the up list
        up = up[::-1]

        #find if the table is in up list or down list
        up_list = False
        for j in range(len(up)):
            if up[j][0] == i[0]:
                up_list = True
                break

        print("table number: ", i[0], "up_list: ", up_list, "up: ", up, "down: ", down)

        #find count negative  count if in up list positive if in down list
        if up_list:
            count = 0
            for j in range(len(up)):
                if up[j][0] == i[0]:
                    count = -(j+1)
                    break

        else:
            count = 0
            for j in range(len(down)):
                if down[j][0] == i[0]:
                    count = j+1
                    break
        
        table_metadata.append((i[0], start_junction, count))

    #find how many rows in the left of the start junction
    left = 0
    for j in range(len(center_points_horizontal)):
        if center_points_horizontal[j][0] < start_point[0]:
            left += 1

    print("left: ", left)

    print(table_metadata)

    # Generate the AVR code
    avr_code = generate_avr_code(left, table_metadata)
    print(avr_code)

    # Save the AVR code to a file named "main.c"
    file_name = Restaurant_name + ".c"
    save_avr_code_to_file(file_name, avr_code)

    #starting horizontal line from strt point to the last table point
    canvas.create_line(points_horizontal[0] if start_point[0] > points_horizontal[0] else start_point[0], start_point[1] + 20 , points_horizontal[-1] if start_point[0] < points_horizontal[-1] else start_point[0], start_point[1] + 20, fill="white", width=4)

    #draw vertical lines
    for i in range(len(points_horizontal)):
        canvas.create_line(points_horizontal[i], (start_point[1] + 20) if (start_point[1] + 20) < points_vertical[i][1] else points_vertical[i][1], points_horizontal[i], points_vertical[i][0] if (start_point[1] + 20) < points_vertical[i][0] else (start_point[1] + 20) , fill="white", width=4)


def prompt_table_info(canvas, table_info_list):
    def add_table():
        try:
            table_number = int(entry_table_number.get())
            table_center_x = int(entry_table_center_x.get())
            table_center_y = int(entry_table_center_y.get())
            table_width = int(entry_table_width.get())
            table_length = int(entry_table_length.get())
            
            table_info = (table_number, table_center_x, table_center_y, table_width, table_length)
            table_info_list.append(table_info)
            
            entry_table_number.delete(0, tk.END)
            entry_table_center_x.delete(0, tk.END)
            entry_table_center_y.delete(0, tk.END)
            entry_table_width.delete(0, tk.END)
            entry_table_length.delete(0, tk.END)
            
            draw_tables(canvas, [table_info])
            messagebox.showinfo("Table Added", f"Table {table_number} added successfully.")
        except ValueError:
            messagebox.showerror("Invalid input", "Please enter valid numbers for all fields.")
    
    def done():
        draw_shortest_path(canvas, table_info_list)
        table_window.destroy()
    
    table_window = tk.Toplevel()
    table_window.title("Enter Table Information")
    
    ttk.Label(table_window, text="Table Number:").grid(row=0, column=0, padx=10, pady=5)
    entry_table_number = ttk.Entry(table_window)
    entry_table_number.grid(row=0, column=1, padx=10, pady=5)
    
    ttk.Label(table_window, text="Table Center X:").grid(row=1, column=0, padx=10, pady=5)
    entry_table_center_x = ttk.Entry(table_window)
    entry_table_center_x.grid(row=1, column=1, padx=10, pady=5)
    
    ttk.Label(table_window, text="Table Center Y:").grid(row=2, column=0, padx=10, pady=5)
    entry_table_center_y = ttk.Entry(table_window)
    entry_table_center_y.grid(row=2, column=1, padx=10, pady=5)
    
    ttk.Label(table_window, text="Table Width:").grid(row=3, column=0, padx=10, pady=5)
    entry_table_width = ttk.Entry(table_window)
    entry_table_width.grid(row=3, column=1, padx=10, pady=5)
    
    ttk.Label(table_window, text="Table Length:").grid(row=4, column=0, padx=10, pady=5)
    entry_table_length = ttk.Entry(table_window)
    entry_table_length.grid(row=4, column=1, padx=10, pady=5)
    
    ttk.Button(table_window, text="Add", command=add_table).grid(row=5, column=0, padx=10, pady=10)
    ttk.Button(table_window, text="Done", command=done).grid(row=5, column=1, padx=10, pady=10)
    
    table_window.transient(root)
    table_window.grab_set()
    root.wait_window(table_window)

def calculate_area():
    global start_point
    global Restaurant_name
    try:
        length = float(entry_length.get())
        width = float(entry_width.get())
        start_point[0] = float(entry_start_x.get())
        start_point[1] = float(entry_start_y.get())
        Restaurant_name = entry_name.get()
        area = length * width

        # Calculate the aspect ratio and set the new window size
        aspect_ratio = width / length
        new_width = 800
        new_height = int(new_width / aspect_ratio)

        # Close the original window
        root.destroy()

        # Create a new window to display the result
        result_window = tk.Tk()
        result_window.title("Area Result")
        result_window.geometry(f"{new_width}x{new_height}")

        # Create a canvas for drawing tables
        canvas = tk.Canvas(result_window, width=new_width, height=new_height)
        canvas.pack()

        # Get table information from the user
        table_info_list = []
        prompt_table_info(canvas, table_info_list)

        # Display the result in the new window
        label_result = tk.Label(result_window, text=f"Restaurant Planner\nTotal Area: {area} square meters")
        label_result.pack(pady=10)

        # Run the new window's main loop
        result_window.mainloop()
    except ValueError:
        messagebox.showerror("Invalid input", "Please enter valid numbers for length and width.")

# Create the main window
root = tk.Tk()
root.title("Area Mapper")
root.geometry("800x600")

# Load the background image
background_image = Image.open("Backdrop.png")

# Resize the image to fit the window size
background_image = background_image.resize((800, 600), Image.ANTIALIAS)

# Convert Image object into Tkinter-compatible image
tk_image = ImageTk.PhotoImage(background_image)

# Create a label with the image
background_label = tk.Label(root, image=tk_image)
background_label.place(x=0, y=0, relwidth=1, relheight=1)


# Create a transparent frame to hold the input fields and button
frame = tk.Frame(root, bg=root.cget('bg'))
frame.pack(expand=True)

label_name = tk.Label(frame, text="Restaurant Name:")
label_name.pack(pady=10)

entry_name= tk.Entry(frame)
entry_name.pack(pady=10)

# Create and place the labels and entries for length and width
label_length = tk.Label(frame, text="Length (meters):")
label_length.pack(pady=10)

entry_length = tk.Entry(frame)
entry_length.pack(pady=10)

label_width = tk.Label(frame, text="Width (meters):")
label_width.pack(pady=10)

entry_width = tk.Entry(frame)
entry_width.pack(pady=10)

label_length = tk.Label(frame, text="Start location x (meters):")
label_length.pack(pady=10)

entry_start_x = tk.Entry(frame)
entry_start_x.pack(pady=10)

label_width = tk.Label(frame, text="Start location y (meters):")
label_width.pack(pady=10)

entry_start_y = tk.Entry(frame)
entry_start_y.pack(pady=10)

# Create and place the calculate button
button_calculate = tk.Button(frame, text="Build Workspace", command=calculate_area)
button_calculate.pack(pady=20)


# Run the application
root.mainloop()
