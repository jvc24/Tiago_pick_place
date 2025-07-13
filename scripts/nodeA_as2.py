    def pick_and_place(self):
        # Initial move has already been called, assuming docking position

        # Block 6
        self.TARGET_TAG_ID = "6"
        self.execute_pick()

        #moving to placement dock
        self.move_to(8, -3.9, 0)
        self.move_to(9, -4, 0)
        self.move_to(9, -4, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)

        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()
        self.execute_place()

        # Block 4
        self.TARGET_TAG_ID = "4"

        #moving to pick dock
        self.move_to(9, -2, -90)
        self.move_to(9, -4, -90)
        self.move_to(9, -4, 180)
        self.move_to(7.8, -4, 180)
        self.move_to(7.8, -4, 90)
        self.move_to(7.8, -3.9, 90)

        self.execute_pick()

        #moving to placement dock
        self.move_to(7.8, -3.9, 0)
        self.move_to(9, -4, 0)
        self.move_to(9, -4, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)

        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()
        self.execute_place()

        # Block 5
        self.TARGET_TAG_ID = "5"

        #moving to pick dock
        self.move_to(9, -2, -90)
        self.move_to(9, -3, -90)
        self.move_to(9, -3, 180)

        self.execute_pick()

        #moving to placement dock
        self.move_to(9, -3, 90)
        self.move_to(9, -2, 90)
        self.move_to(9, -2, 180)

        self.TARGET_TAG_ID = "10" #Change filter target to placement table apriltag
        self.compute_place_point()
        self.execute_place()