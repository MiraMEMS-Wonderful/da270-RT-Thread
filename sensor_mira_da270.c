
#include "sensor_mira_da270.h"

#define DBG_ENABLE
#define DBG_LEVEL DBG_LOG
#define DBG_SECTION_NAME  "sensor.mira.da270"
#define DBG_COLOR
#include <rtdbg.h>

#define GRAVITY_EARTH (9.80665f)

static void rt_delay_ms(uint32_t period)
{
    rt_thread_mdelay(period);
}

static int8_t rt_i2c_write_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_WR | RT_I2C_NO_START;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static int8_t rt_i2c_read_reg(void *intf_ptr, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t len)
{
    rt_uint8_t tmp = reg;
    struct rt_i2c_msg msgs[2];

    msgs[0].addr  = addr;             /* Slave address */
    msgs[0].flags = RT_I2C_WR;        /* Write flag */
    msgs[0].buf   = &tmp;             /* Slave register address */
    msgs[0].len   = 1;                /* Number of bytes sent */

    msgs[1].addr  = addr;             /* Slave address */
    msgs[1].flags = RT_I2C_RD;        /* Read flag */
    msgs[1].buf   = data;             /* Read data pointer */
    msgs[1].len   = len;              /* Number of bytes read */

    if (rt_i2c_transfer(intf_ptr, msgs, 2) != 2)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}

static struct da270_dev *_da270_create(struct rt_sensor_intf *intf)
{
    struct da270_dev *_da270_dev = RT_NULL;
    struct rt_i2c_bus_device *i2c_bus_dev = RT_NULL;
	
    int8_t rslt = DA270_OK;
    struct da270_sensor_conf conf;

    i2c_bus_dev = (struct rt_i2c_bus_device *)rt_device_find(intf->dev_name);
    if (i2c_bus_dev == RT_NULL)
    {
        LOG_E("can not find device %s", intf->dev_name);
        return RT_NULL;
    }

    _da270_dev = rt_calloc(1, sizeof(struct da270_dev));
    if (_da270_dev == RT_NULL)
    {
        LOG_E("da270 dev memory allocation failed");
        return RT_NULL;
    }

    _da270_dev->dev_id   = (rt_uint32_t)(intf->user_data) & 0xff;
    _da270_dev->intf     = DA270_I2C_INTF;
    _da270_dev->intf_ptr = i2c_bus_dev;
    _da270_dev->read     = rt_i2c_read_reg;
    _da270_dev->write    = rt_i2c_write_reg;
    _da270_dev->delay_ms = rt_delay_ms;

    rslt = da270_init(_da270_dev);
    if (rslt == DA270_OK)
    {
        rslt = da270_soft_reset(_da270_dev);	//TODO, how to do soft reset?

        /* Select the type of configuration to be modified */
        conf.type = DA270_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        rslt = da270_get_sensor_conf(&conf, 1, _da270_dev);

        /* Modify the desired configurations as per macros
         * available in da270_defs.h file */
        conf.param.accel.odr = DA270_ODR_125HZ;
        conf.param.accel.range = DA270_2G_RANGE;

        da270_set_power_mode(DA270_SLEEP_MODE, _da270_dev);

        return _da270_dev;
    }
    else
    {
        LOG_E("da270 init failed, %d", rslt);
        rt_free(_da270_dev);
        return RT_NULL;
    }
}

static rt_err_t _da270_set_odr(rt_sensor_t sensor, rt_uint16_t odr)
{
    struct da270_dev *_da270_dev = sensor->parent.user_data;
    struct da270_sensor_conf conf;
    uint8_t odr_ctr;

    if (odr == 1)
        odr_ctr = DA270_ODR_1_HZ;
    else if (odr <= 2)
        odr_ctr = DA270_ODR_1_95HZ;
    else if (odr <= 4)
        odr_ctr = DA270_ODR_3_9HZ;
    else if (odr <= 8)
        odr_ctr = DA270_ODR_7_81HZ;
    else if (odr <= 16)
        odr_ctr = DA270_ODR_15_63HZ;
    else if (odr <= 32)
        odr_ctr = DA270_ODR_31_25HZ;
    else if (odr <= 63)
        odr_ctr = DA270_ODR_62_5HZ;
    else if (odr <= 125)
        odr_ctr = DA270_ODR_125HZ;
    else if (odr <= 250)
        odr_ctr = DA270_ODR_250HZ;
    else if (odr <= 500)
        odr_ctr = DA270_ODR_500HZ;
    else
        odr_ctr = DA270_ODR_1000HZ;

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        conf.type = DA270_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        da270_get_sensor_conf(&conf, 1, _da270_dev);

        conf.param.accel.odr = odr_ctr;

        /* Set the desired configurations to the sensor */
        da270_set_sensor_conf(&conf, 1, _da270_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _da270_set_range(rt_sensor_t sensor, rt_uint16_t range)
{
    struct da270_dev *_da270_dev = sensor->parent.user_data;

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        struct da270_sensor_conf conf;
        uint8_t range_ctr;

        if (range <= 2000)
            range_ctr = DA270_2G_RANGE;
        else if (range <= 4000)
            range_ctr = DA270_4G_RANGE;
        else if (range <= 8000)
            range_ctr = DA270_8G_RANGE;
        else
            range_ctr = DA270_16G_RANGE;

        conf.type = DA270_ACCEL;

        /* Get the accelerometer configurations which are set in the sensor */
        da270_get_sensor_conf(&conf, 1, _da270_dev);

        conf.param.accel.range = range_ctr;

        /* Set the desired configurations to the sensor */
        da270_set_sensor_conf(&conf, 1, _da270_dev);
        return RT_EOK;
    }

    return RT_EOK;
}

static rt_err_t _da270_set_power(rt_sensor_t sensor, rt_uint8_t power)
{
    struct da270_dev *_da270_dev = sensor->parent.user_data;
    int8_t rslt = 0;

    if (power == RT_SENSOR_POWER_DOWN)
    {
        rslt = da270_set_power_mode(DA270_SLEEP_MODE, _da270_dev);
    }
    else if ((power == RT_SENSOR_POWER_NORMAL)||(power == RT_SENSOR_POWER_LOW))
    {
        rslt = da270_set_power_mode(DA270_NORMAL_MODE, _da270_dev);
    }
    else
    {
        LOG_W("Unsupported mode, code is %d", power);
        return -RT_ERROR;
    }

    return rslt;
}

static rt_size_t da270_fetch_data(struct rt_sensor_device *sensor, void *buf, rt_size_t len)
{
    struct da270_dev *_da270_dev = sensor->parent.user_data;
    struct rt_sensor_data *data = buf;

    if (sensor->info.type == RT_SENSOR_CLASS_ACCE)
    {
        struct da270_sensor_data comp_data;
        da270_get_accel_data(&comp_data, _da270_dev);

        data->type = RT_SENSOR_CLASS_ACCE;
        data->data.acce.x = comp_data.x;
        data->data.acce.y = comp_data.y;
        data->data.acce.z = comp_data.z;
        data->timestamp = rt_sensor_get_ts();
    }

    return 1;
}

static rt_err_t da270_control(struct rt_sensor_device *sensor, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    struct da270_dev *_da270_dev = sensor->parent.user_data;

    switch (cmd)
    {
    case RT_SENSOR_CTRL_GET_ID:
        *(rt_uint8_t *)args = _da270_dev->chip_id;
        break;
    case RT_SENSOR_CTRL_SET_ODR:
        result = _da270_set_odr(sensor, (rt_uint32_t)args & 0xffff);
        break;
    case RT_SENSOR_CTRL_SET_RANGE:
        result = _da270_set_range(sensor, (rt_uint32_t)args);
        break;
    case RT_SENSOR_CTRL_SET_POWER:
        result = _da270_set_power(sensor, (rt_uint32_t)args & 0xff);
        break;
    default:
        return -RT_EINVAL;
    }

    return result;
}

static struct rt_sensor_ops sensor_ops =
{
    da270_fetch_data,
    da270_control
};

int rt_hw_da270_init(const char *name, struct rt_sensor_config *cfg)
{
    rt_int8_t result;
    rt_sensor_t sensor_acce = RT_NULL;
    struct da270_dev *_da270_dev = RT_NULL;

#ifdef PKG_USING_DA270_ACCE
    /* accelerometer sensor register */
    {
        sensor_acce = rt_calloc(1, sizeof(struct rt_sensor_device));
        if (sensor_acce == RT_NULL)
            return -RT_ERROR;

        sensor_acce->info.type       = RT_SENSOR_CLASS_ACCE;
        sensor_acce->info.vendor     = RT_SENSOR_VENDOR_STM;
        sensor_acce->info.model      = "da270_acce";
        sensor_acce->info.unit       = RT_SENSOR_UNIT_MG;
        sensor_acce->info.intf_type  = RT_SENSOR_INTF_I2C;
        sensor_acce->info.range_max  = 16000;
        sensor_acce->info.range_min  = 2000;
        sensor_acce->info.period_min = 1;

        rt_memcpy(&sensor_acce->config, cfg, sizeof(struct rt_sensor_config));
        sensor_acce->ops = &sensor_ops;

        result = rt_hw_sensor_register(sensor_acce, name, RT_DEVICE_FLAG_RDWR, RT_NULL);
        if (result != RT_EOK)
        {
            LOG_E("device register err code: %d", result);
            rt_free(sensor_acce);
            return -RT_ERROR;
        }
    }

    LOG_I("acc sensor init success");
#endif

    _da270_dev = _da270_create(&cfg->intf);
    if (_da270_dev == RT_NULL)
    {
        LOG_E("sensor create failed");
        return -RT_ERROR;
    }

    sensor_acce->parent.user_data = _da270_dev;

    return RT_EOK;
}
