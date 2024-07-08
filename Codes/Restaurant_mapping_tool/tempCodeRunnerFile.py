nvas.create_rectangle(table_top_left_x, table_top_left_y, table_bottom_right_x, table_bottom_right_y, fill="", outline="black")

        # Create a pattern using the texture image
        canvas.create_image(table_top_left_x, table_top_left_y, image=texture_image, anchor="nw")
