from tile import tile
from board import board
from coordinate import coordinate

import unittest
import random
class tile_test_methods(unittest.TestCase):


    tile1 = tile()
    tile2 = tile()


    existing_vals = tile1.get_tile_mapping()
    existing_val = existing_vals[existing_vals.keys()[0]]
    new_val = str(random.randint(0,100))
    while new_val in existing_vals.values(): new_val =  str(random.randint(0,100))
    bad_setTo_val = existing_vals[existing_vals.keys()[1]]
    bad_existingVal = 10

    def test_set_not_existing_var(self):
        response = self.tile1.set_tile(self.bad_existingVal, current_mapping=self.new_val)
        self.assertEqual(response,False,"Can't set var that does not exist")
        response = self.tile1.set_tile(self.bad_existingVal, current_mapping=self.new_val)
        self.assertEqual(response,False,"Can't set var that does not exist")

    def test_setTo_existing_val(self):
        response = self.tile1.set_tile(self.existing_val,current_mapping=self.bad_setTo_val)
        self.assertEqual(response,False,"Can't set a value the already exist in other val")

    def test_set_var(self):
    #   testing var set
        var_key = None
        current_keys = self.existing_vals
        for key in current_keys:
            if current_keys[key] == self.existing_val:
                var_key = key
                print var_key
                break

        self.assertNotEqual(var_key,None,"existing_val error - does not exist")
        print (self.existing_val,self.new_val)
        print(self.tile1.get_tile_mapping())
        flag = self.tile1.set_tile(self.existing_val,self.new_val)
        self.assertEqual(flag,True)


        #   Fix: those two tests
        #response = (self.tile1.get_tile_mapping())[var_key]
        #self.assertEqual(response,self.new_val)


        #response = self.tile2.get_tile_mapping()[key]
        #self.assertEqual(response,self.new_val)

    def test_set_to_same_val(self):
        response = self.tile1.set_tile(self.existing_val, self.existing_val)
        self.assertEqual(response,True)

    def test_isLegal_val(self):
        response = self.tile1.isVal_legal(self.bad_existingVal)
        self.assertEqual(response,False)

        response = self.tile1.isVal_legal(self.existing_val)
        self.assertEqual(response,True)

    def test_direction_identifier(self):
        val = self.tile1.get_car_directions().values()[0]
        response = self.tile1.isDirection_available(val)
        self.assertEqual(response,True)
        randomized= random.randrange(0,100)
        while randomized in self.tile1.get_car_directions().keys():
            randomized= random.randrange(0,100)
        response = self.tile1.isDirection_available(randomized)
        self.assertEqual(response,False)

    def test_isAvailable(self):
        response = self.tile1.isVal_available(self.existing_val)
        self.assertEqual(response,True)

        response = self.tile1.isVal_available(self.bad_existingVal)
        self.assertEqual(response,False)

class board_test_methods(unittest.TestCase):
    #sampleBoard1 = board()

    def test_boardSize(self):
        xVal = 5
        yVal = 3
        test_board = board(boardSize={'x':xVal,'y':yVal})
        board_size = test_board.get_board_size()
        self.assertEqual(xVal,board_size['x'])
        self.assertEqual(yVal,board_size['y'])

    def test_illegal_boardSize(self):
        illegalVal = -1
        legalVal = 2
        try:
            board(boardSize={'x':illegalVal,'y':legalVal})
        except:
            self.assertTrue(True)

        try:
            board(boardSize={'x':legalVal,'y':illegalVal})
        except:
            self.assertTrue(True)

    def test_car_initial_placement(self):
        try:
            #   Assuming board default sizes are x:10 y:10
            board(carPlacement={'x':11,'y':12})
            self.assertFalse(True)

        except:
            self.assertTrue(True)

        try:
            board(boardSize={'x':3,'y':3},carPlacement={'x':3,'y':3})
            self.assertFalse(True)
        except:
            self.assertTrue(True)

        try:
            board(boardSize={'x':3,'y':3},carPlacement={'x':1,'y':1})
            self.assertTrue(True)
        except:

            self.assertFalse(True)

    def test_xy_crop(self):

        x_original = 5
        y_original = 5
        sampleBoard = board(boardSize={'x':x_original,'y':y_original})
        try:
            sampleBoard.get_xy_map(-1,2)
            self.assertTrue(False)
        except:
            self.assertTrue(True)


        try:
            sampleBoard.get_xy_map(1,-1)
            self.assertTrue(False)
        except:
            self.assertTrue(True)

        x_param = 1
        y_param = 2
        cropped_board = sampleBoard.get_xy_map(x_param,y_param)
        self.assertEqual(cropped_board.get_board_size()['x'],x_param*2+1)
        self.assertEqual(cropped_board.get_board_size()['y'],y_param*2+1)

class coordinate_test_methods(unittest.TestCase):

    c_instance = None


    def test_creation(self):
        self.c_instance = coordinate(1,1)
        self.assertTrue(isinstance(self.c_instance,coordinate),"coordinate was not created properly")
        self.assertEqual(self.c_instance.get_y(),1)
        self.assertEqual(self.c_instance.get_x(),1)
    def test_creation_with_wrong_val(self):
        self.c_instance = coordinate('a','a')
        self.assertEqual(self.c_instance.get_y(),0)
        self.assertEqual(self.c_instance.get_x(),0)
    def test_coordinate_setting(self):
        self.c_instance = coordinate(1,1)
        self.c_instance.set_x(5)
        self.c_instance.set_y(6)
        self.assertEqual(self.c_instance.get_y(),6)
        self.assertEqual(self.c_instance.get_x(),5)
if __name__ == '__main__':
    unittest.main()