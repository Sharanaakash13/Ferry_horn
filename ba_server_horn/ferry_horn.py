import serial 
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String

class HornSubscriber(Node):

    # Defining the serial port connected to relay board 
    Serial_port = '/dev/ttyUSB0' 

    #byte0
    # Command for the relay board in integer
    cmd_single = 6  # command for set single (Switch on relay without changing other output) 
    cmd_delete = 7  # command for set single (Switch off relay without changing other output) 

    # byte1
    # Address of the relay board
    address_relay = 1 # Only one relay board (integer)

    #byte2
    # Defining the data to the relay board 
    Horn = 0b00100000   # relay output 3 for horn

    #byte3
    # XOR sum 
    set_horn_XOR = (cmd_single^address_relay^Horn) # XOR for activating horn output
    del_horn_XOR = (cmd_delete^address_relay^Horn) # XOR for deactivating horn output

    def __init__(self):

        # Node for the Ferry light
        super().__init__('horn_subscriber')

        # Checking the serial port 
        if not os.path.exists(self.Serial_port):
            self.get_logger().error("Serial port does not exists :" + self.Serial_port)
            self.get_logger().info("Command not sent to the relay board")
            rclpy.shutdown()

        # Start the serial communication
        self.ser = serial.Serial(
            port=self.Serial_port, 
            baudrate=19200, 
            parity= serial.PARITY_NONE, 
            stopbits = serial.STOPBITS_ONE, 
            bytesize= serial.EIGHTBITS,
            timeout=1)   # Serial data configuration based on Conrad manual 
        
        # Checking the serial port is open 
        try: 
			self.ser.isOpen()==False
            self.ser.open()      # Opens the serial port

		except:
			print('Serial port is already open')

        # Creating Boolean subscription 
        self.subscription = self.create_subscription(Bool,'/Bool_topic', self.send_serial_cmd, 10)
        self.get_logger().info("Serial communication started")
        self.subscription # prevent unused variable warning

        # Creating String publisher (Publishing the status)
        self.publisher = self.create_publisher(String,'/String_topic',10)

        # Timer callback for publisher
        self.time_period = 1 # seconds 
        self.timer = self.create_timer(self.time_period, self.read_serial)

    def send_serial_cmd(self,msg):

        # Relay board switch on Horn if the message is true.
        if msg.data == True:
            self.get_logger().info("Initiating serial communication")
            
            # Creating a byte array for docking light 
            self.single = [cmd_single, address_relay, Horn,set_horn_XOR]
            self.bytearray_1 = bytearray(self.single)

            # Sending the signal
            self.get_logger().info("Sending serial : 6-1-00100000-XOR")
            self.send_serial(self.bytearray_1)

        # Deactivates the horn with boolean message as false
        else :
            self.get_logger().info("Switching off the horn")
            
            # Creating a byte array for deactivating horn 
            self.delete = [cmd_delete_bit, address_relay_bit, del_horn,del_horn_XOR]
            self.bytearray_2 = bytearray(self.delete)

            # Sending the signal
            self.get_logger().info("Sending serial : 7-1-00100000-XOR")
            self.send_serial(self.bytearray_2)
   
    def send_serial(self, send):
        # Sending serial communication to the relay board 
        self.ser.write(send)

    def read_serial(self):
        # Expected response from relay board
		expected_response_single = 249            # Expected byte 0 response for single 
		expected_response_delete = 248            # Expected byte 0 response for delete 

        # Read serial from the relay board
        self.response_serial = self.ser.read(1)     #Byte 0 response from serial 
        
        # Decoding the response command  
        self.decode= int.from_bytes(self.response_serial)
        print('Response : ' + self.decode)

        # Error response 
        if self.decode == expected_response_single:
            self.var = 1
            self.pub_status()       # publishing the status 
            rclpy.shutdown()

        # Horn activated 
        elif self.serial_response ==  expected_response_delete:
            self.var = 2 
            self.pub_status()       # publishing the status 
        
        # Horn deactivated 
        else: 
            self.var = 3
            self.pub_status()       # publishing the status 

    def pub_status(self):
        # Defining message type for publisher 
        status = String()

        # Condition for publisher 
        if self.var == 1:            
            status.data = "Horn activated."
            self.get_logger().info("Relay board : Horn switched on")
       
        elif self.var == 2:                              
            status.data = 'Horn deactivated.'
            self.get_logger().info("Relay board : Horn switched off")
        
        else:                                       # if relay board has error
            status.data = "Something went wrong....."
            self.get_logger().info("Error : Something went wrong!!")

        self.publisher.publish(status)    # Publishing status to the same topic

def main(args=None):
    rclpy.init(args=args)
    topic_subscriber = Subscriber()
    rclpy.spin(topic_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()