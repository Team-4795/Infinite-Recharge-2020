package frc.robot;

import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
  private java.util.Timer executor;
  private static final long THREAD_PERIOD = 20; // ms - max poll rate on sensor.
  
  public static final byte CS_ADDRESS = 0x39;

  private static ColorSensor instanceOnboard;
  private static ColorSensor instanceMXP;
  private I2C cs;
  private volatile ColorData color;

  public enum reg_t {
    CS_ENABLE (0x00),
    CS_ATIME  (0x01),
    CS_CDATA  (0x14),
    CS_RDATA  (0x16),
    CS_GDATA  (0x18),
    CS_BDATA  (0x1A);

    private final int val;

    reg_t(int val) {
      this.val = val;
    }

    public int getVal() {
      return 0x80 | val;
    }
  };
  
  public class ColorData {
    public double clear;
    public double red;
    public double green;
    public double blue;
  }
  
  private ColorSensor(I2C.Port port) {
    cs = new I2C(port, CS_ADDRESS);

    write8(reg_t.CS_ENABLE, (byte) (1 | 2));
    write8(reg_t.CS_ATIME, (byte) 0xEC);

    executor = new java.util.Timer();
    executor.schedule(new ColorSensorUpdateTask(this), 0L, THREAD_PERIOD);
  }

  public static ColorSensor getInstanceOnboard() {
    if (instanceOnboard == null) {
      instanceOnboard = new ColorSensor(I2C.Port.kOnboard);
    }
    return instanceOnboard;
  }

  public static ColorSensor getInstanceMXP() {
    if (instanceMXP == null) {
      instanceMXP = new ColorSensor(I2C.Port.kMXP);
    }
    return instanceMXP;
  }

  private void update() {
    ColorData colorData = new ColorData();

    colorData.clear = (read16(reg_t.CS_CDATA) & 0xffff) / 65536.0;
    colorData.red = (read16(reg_t.CS_RDATA) & 0xffff) / 65536.0;
    colorData.green = (read16(reg_t.CS_GDATA) & 0xffff) / 65536.0;
    colorData.blue = (read16(reg_t.CS_BDATA) & 0xffff) / 65536.0;

    color = colorData;
  }

  public ColorData getColor() {
    return color;
  }
  
  private boolean write8(reg_t reg, byte value) {
    return cs.write(reg.getVal(), value);
  }

  // private byte read8(reg_t reg) {
  //   byte[] vals = new byte[1];
  //   readLen(reg, vals);
  //   return vals[0];
  // }

  private short read16(reg_t reg) {
    byte[] vals = new byte[2];
    readLen(reg, vals);
    return (short) ((vals[0] & 0xff) | ((vals[1] & 0xff) << 8));
  }

  // return true if successful
  private boolean readLen(reg_t reg, byte[] buffer) {
    if (buffer == null || buffer.length < 1) {
      return false;
    }
    return !cs.read(reg.getVal(), buffer.length, buffer);
  }
  
  private class ColorSensorUpdateTask extends TimerTask {
    private ColorSensor cs;

    private ColorSensorUpdateTask(ColorSensor cs) {
      if (cs == null) {
        throw new NullPointerException("ColorSensor pointer null");
      }
      this.cs = cs;
    }

    public void run() {
      cs.update();
    }
  }
}