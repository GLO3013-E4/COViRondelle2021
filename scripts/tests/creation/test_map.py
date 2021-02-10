#test create nodes
#test connect nodes
#test add cushion
#test create obstacles
#test create pucks
#test create start_node
#test create end_node
#test create add pickup cushion
#test get start node
#test get end node
#test render map


# MOCKS MOCKS MOCKS

from scripts.creation.Map import Map

from PIL import Image




class TestMap:
    @classmethod
    def setup_class(cls):
        cls.AN_IMAGE_WIDTH = 300
        cls.AN_IMAGE_HEIGHT = 300
        cls.AN_IMAGE = Image.new("RGB", (cls.AN_IMAGE_WIDTH, cls.AN_IMAGE_HEIGHT))
        cls.SOME_OBSTACLES = [
            ()
        ]
        cls.SOME_PUCKS = [
            ()
        ]
        cls.AN_ENDING_POSITION = ()
        cls.A_STARTING_POSITION = ()
        cls.A_NODE_SIZE = 25
        cls.A_SAFETY_CUSHION = 20
        cls.A_ROBOT_WIDTH = 100
        cls.AN_OBSTACLE_WIDTH = 40
        cls.A_PUCK_WIDTH = 25

    def setup_method(self):
        self.Map = Map(self.AN_IMAGE, self.SOME_OBSTACLES, self.SOME_PUCKS, self.A_STARTING_POSITION, self.AN_ENDING_POSITION, self.A_NODE_SIZE, self.A_SAFETY_CUSHION, self.A_ROBOT_WIDTH, self.AN_OBSTACLE_WIDTH, self.A_PUCK_WIDTH)













