sdk_inc(motor motor/drivers/encos motor/drivers/cih408 imu)
sdk_src(
    motor/motor_driver.cpp
    motor/motor_driver_binding.cpp
    motor/motor_instance.cpp
    motor/drivers/encos/encos_driver.cpp
    motor/drivers/encos/encos_session.cpp
    motor/drivers/encos/encos_parameter_transactions.cpp
    motor/drivers/cih408/cih408_driver.cpp
    imu/imu_device.cpp
)
