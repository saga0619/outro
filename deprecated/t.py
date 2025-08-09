from pymodbus.client import ModbusSerialClient as mbus
import time

#from pymodbus.client import AsyncModbusSerialClient as mbus
from pymodbus import (
    ModbusException,
    pymodbus_apply_logging_config,
)

id =1
class driver:
    def __init__(self):
        self.ready= 0
        self.run = 0 
        self.error =0 
        self.was_home_fin=0
        self.inposition =0
        self.torque=0
        self.qdot=0
        self.q=0
        self.cnt2rad=0.00001
        self.v=0
        self.p=0
        
    def connect_mbus(self):
        self.client = mbus("COM3",baudrate=38400)
        pymodbus_apply_logging_config("DEBUG")
    
        if not self.client.connected:
            self.client.connect()
        else:
            pass

    def get_data(self):
        if self.client.connected: 
            try: 
                rr = self.client.read_holding_registers(address=0x0b05,count=3,slave=id)
            except ModbusException as exc:
                print(f"Received ModbusException({exc})")
                return
            if rr.isError():
                print(f"Received exception from device ({rr})")
                return
            self.ready= rr.registers[0]&1
            self.run= (rr.registers[0]>>1)&1
            self.error= (rr.registers[0]>>2)&1
            self.was_home_fin = (rr.registers[0]>>3)&1
            self.inposition = (rr.registers[0]>>4)&1
            self.qdot=self.client.convert_from_registers(rr.registers,data_type=self.client.DATATYPE.INT16)[1]
            self.torque=self.client.convert_from_registers(rr.registers,data_type=self.client.DATATYPE.INT16)[2]
            self.v=self.qdot*self.cnt2rad
            try: 
                rr2 = self.client.read_holding_registers(address=0x602c,count=2,slave=id)
            except ModbusException as exc:
                print(f"Received ModbusException({exc})")
                return
            if rr2.isError():
                print(f"Received exception from device ({rr})")
                return
            self.q = self.client.convert_from_registers(rr2.registers,data_type=self.client.DATATYPE.INT32)
            self.p=self.q*self.cnt2rad
            print(f"got position {self.q} {self.qdot} {self.ready}  {self.run} {self.error}")

if __name__ == "__main__":
    
    m1=driver()
    m1.connect_mbus()

    cnt = 0

    start_time = time.time()
    
    while True:
        m1.get_data()
        cnt += 1
        
        time.sleep(0.1)

        print(f"{cnt} : {time.time()-start_time}")
        # if KeyboardInterrupt:
        #     break

    print("terminate test code")    