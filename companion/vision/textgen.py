class Logger:
    def __init__(self, shared_state):
        self.shared_state = shared_state

    def run(self):
        for i in range(50): 
            if self.shared_state.get_ransac_fired() and self.shared_state.get_yolo_fired():
                string = self.shared_state.__repr__()
                self.shared_state.clear_ransac_yolo_fired()

                try:
                    with open("targets.txt", 'a') as f:
                        f.write(f"Color: {string} \n")

                except Exception as e:
                    print(f"Exception while writing to text file: {e}")

                return

