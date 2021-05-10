import serial
import numpy as np
import msgpack



def main():
    port = "COM8"
    baud_rate = 2000000
    batch_size = 200
    data_arr = np.empty((batch_size,10),dtype=float)
    np.set_printoptions(linewidth=np.inf)
    try:
        ser = serial.Serial(timeout=1)
        ser.baudrate = baud_rate
        ser.port = port
        ser.open()
        # x = ser.read()
        k = 0
        while(1):
            # ser.reset_input_buffer()
            raw_data = ser.readline()
            data = np.asarray(raw_data[:-1].split(b" "),dtype=float)
            data_arr[k%batch_size] = data
            if k>0 and k%batch_size==0:
                print(data_arr.mean(axis=0))
            k+=1

    except Exception as e:
        print(e)
        ser.close()
        return
    ser.close()


if __name__ == '__main__':
    main()