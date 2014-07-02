# location of the Python header file
PYTHON_VERSION = 2.6
PYTHON_INCLUDE = /usr/include/python$(PYTHON_VERSION)
PYTHON_LIB = python$(PYTHON_VERSION)

ROBOARD_INCLUDE = /home/blimp/Desktop/playground/roboard/libsrc
ROBOARD_LIB_DIR = /home/blimp/Desktop/playground/roboard
ROBOARD_LIB = RBIO
# location of the Boost Python include files and library
BOOST_INC = /usr/include
BOOST_LIB = /usr/lib

TARGET = boost_roboard

# .so file is the file that python will import as a module
$(TARGET).so: $(TARGET).o
	g++ -shared -Wl,--export-dynamic $(TARGET).o -L$(BOOST_LIB) -lboost_python -L$(ROBOARD_LIB_DIR) -l$(ROBOARD_LIB) -lpython$(PYTHON_VERSION) -o $(TARGET).so

$(TARGET).o: $(TARGET).cpp
	g++ -I$(PYTHON_INCLUDE) -I$(BOOST_INC) -fPIC -c $(TARGET).cpp -I$(ROBOARD_INCLUDE)
#	g++ -I$(PYTHON_INCLUDE) -I$(BOOST_INC) $(TARGET).cpp -I$(ROBOARD_INCLUDE) -L$(ROBOARD_LIB_DIR) -l$(ROBOARD_LIB) -l$(PYTHON_LIB)
