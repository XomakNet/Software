## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup, find_packages

# fetch values from package.xml
setup_args = {
    "name": "adafruit_drivers_py",
    "packages": ['Adafruit_ADS1x15', 'Adafruit_GPIO','Adafruit_I2C','Adafruit_LSM303','Adafruit_MotorHAT','Adafruit_PWM_Servo_Driver','Gyro_L3GD20'],
    "package_dir": {'': 'include'},
}

setup(**setup_args)
