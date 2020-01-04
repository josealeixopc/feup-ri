import subprocess, threading
import image_similarity

class Command(object):
    def __init__(self, cmd):
        self.cmd = cmd
        self.process = None

    def run(self, timeout):
        def target():
            print ('Thread started for command: "{}"').format(self.cmd)
            self.process = subprocess.Popen(self.cmd, shell=True)
            self.process.communicate()
            print ('Thread finished for command: "{}"').format(self.cmd)

        thread = threading.Thread(target=target)
        thread.start()

        thread.join(timeout)
        if thread.is_alive():
            print ('Terminating process for command: "{}"').format(self.cmd)
            self.process.terminate()
            thread.join()
        print ('Process "{}" terminated with code: {}'.format(self.cmd, self.process.returncode))

def compare_current_map_to_actual_map(actual_map_file):
    # Run command to save current map in \map topic to a file
    current_map_file_location = "/tmp/my_map"

    command=Command("rosrun map_server map_saver -f {}".format(current_map_file_location))
    command.run(timeout=10)

    image_similarity.compare_images(current_map_file_location + ".pgm", actual_map_file, image1_abs_path=True)


if __name__ == "__main__":
    compare_current_map_to_actual_map("turtlebot3_world_map.pgm")