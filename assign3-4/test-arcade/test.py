"""
Starting Template

Once you have learned how to use classes, you can begin your program with this
template.

If Python and Arcade are installed, this example can be run from the command line with:
python -m arcade.examples.starting_template
"""
import arcade
import pymunk
import timeit

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
SCREEN_TITLE = "Test"


class PymunkSprite(arcade.Sprite):
    """
    We need a Sprite and a Pymunk physics object. This class blends them
    together.
    """
    def __init__(self,
                 filename,
                 center_x=0,
                 center_y=0,
                 scale=1,
                 mass=DEFAULT_MASS,
                 moment=None,
                 friction=DEFAULT_FRICTION,
                 body_type=pymunk.Body.DYNAMIC):

        super().__init__(filename, scale=scale, center_x=center_x, center_y=center_y)

        width = self.texture.width * scale
        height = self.texture.height * scale

        if moment is None:
            moment = pymunk.moment_for_box(mass, (width, height))

        self.body = pymunk.Body(mass, moment, body_type=body_type)
        self.body.position = pymunk.Vec2d(center_x, center_y)

        self.shape = pymunk.Poly.create_box(self.body, (width, height))
        self.shape.friction = friction

class MyGame(arcade.Window):
    """
    Main application class.

    NOTE: Go ahead and delete the methods you don't need.
    If you do need a method, delete the 'pass' and replace it
    with your own code. Don't leave 'pass' in this program.
    """

    def __init__(self, width, height, title):
        super().__init__(width, height, title)

        arcade.set_background_color(arcade.color.AMAZON)

        # If you have sprite lists, you should create them here,
        # and set them to None

        # -- Pymunk
        self.space = pymunk.Space()
        # self.space.iterations = 35
        self.space.gravity = (0,0)

        # Timings
        self.draw_time = 0
        self.processing_time = 0

    def setup(self):
        # Create your sprites and sprite lists here
        body = pymunk.Body(1,1666)  # Create a Body with mass and moment
        body.position = 50,100      # Set the position of the body
        shape = pymunk.Circle(body, 10) # Create a box shape and attach to body
        self.space.add(body, shape)       # Add both body and shape to the simulation


    def on_draw(self):
        """
        Render the screen.
        """

        # This command should happen before we start drawing. It will clear
        # the screen to the background color, and erase what we drew last frame.
        arcade.start_render()

        # Start timing how long this takes
        draw_start_time = timeit.default_timer()

        # Call draw() on all your sprite lists below

        # Display timings
        output = f"Processing time: {self.processing_time:.3f}"
        arcade.draw_text(output, 20, SCREEN_HEIGHT - 20, arcade.color.WHITE, 12)

        output = f"Drawing time: {self.draw_time:.3f}"
        arcade.draw_text(output, 20, SCREEN_HEIGHT - 40, arcade.color.WHITE, 12)

        self.space.debug_dr

        self.draw_time = timeit.default_timer() - draw_start_time

    def on_update(self, delta_time):
        """
        All the logic to move, and the game logic goes here.
        Normally, you'll call update() on the sprite lists that
        need it.
        """
        start_time = timeit.default_timer()

        # Update physics
        # Use a constant time step, don't use delta_time
        # See "Game loop / moving time forward"
        # http://www.pymunk.org/en/latest/overview.html#game-loop-moving-time-forward
        self.space.step(1 / 60.0)

        self.processing_time = timeit.default_timer() - start_time

    def on_key_press(self, key, key_modifiers):
        """
        Called whenever a key on the keyboard is pressed.

        For a full list of keys, see:
        http://arcade.academy/arcade.key.html
        """
        pass

    def on_key_release(self, key, key_modifiers):
        """
        Called whenever the user lets off a previously pressed key.
        """
        pass

    def on_mouse_motion(self, x, y, delta_x, delta_y):
        """
        Called whenever the mouse moves.
        """
        pass

    def on_mouse_press(self, x, y, button, key_modifiers):
        """
        Called when the user presses a mouse button.
        """
        pass

    def on_mouse_release(self, x, y, button, key_modifiers):
        """
        Called when a user releases a mouse button.
        """
        pass


def main():
    """ Main method """
    game = MyGame(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_TITLE)
    game.setup()
    arcade.run()


if __name__ == "__main__":
    main()