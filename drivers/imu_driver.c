/*
 * bPuppy IMU — MPU9250 + AK8963, 最小可用版
 * SDA=GPIO3, SCL=GPIO14, I2C Master 模式, 100kHz
 */
#include "imu_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <string.h>
#include <math.h>

static const char *TAG = "imu";

#define MPU_WHO_AM_I         0x75
#define MPU_PWR_MGMT_1       0x6B
#define MPU_CONFIG           0x1A
#define MPU_GYRO_CONFIG      0x1B
#define MPU_ACCEL_CONFIG     0x1C
#define MPU_ACCEL_CONFIG2    0x1D
#define MPU_INT_PIN_CFG      0x37
#define MPU_USER_CTRL        0x6A
#define MPU_I2C_MST_CTRL     0x24
#define MPU_SLV0_ADDR        0x25
#define MPU_SLV0_REG         0x26
#define MPU_SLV0_CTRL        0x27
#define MPU_SLV0_DO          0x63
#define MPU_EXT_SENS_DATA    0x49
#define MPU_ACCEL_XOUT_H     0x3B

#define AK8963_ADDR          0x0C
#define AK8963_WHO_AM_I      0x00
#define AK8963_ST1           0x02
#define AK8963_CNTL1         0x0A
#define AK8963_CNTL2         0x0B

#define H_RESET       0x80
#define CLKSEL_PLL    0x01
#define I2C_MST_EN    0x20
#define I2C_MST_RST   0x02
#define SLV0_EN       0x80
#define AK8963_HOFL   0x08

#define ACCEL_SCALE  4096.0f
#define GYRO_SCALE   65.5f
#define MAG_SCALE    0.15f
#define TEMP_SCALE   333.87f
#define G_MPS2       9.81f
#define DEG2RAD      0.0174533f

#define IMU_CAL_NS   "imu_cal"
#define IMU_TASK_STACK  4096
#define IMU_TASK_PRIO   7
#define IMU_TASK_HZ     100
#define IMU_TASK_PERIOD (1000/IMU_TASK_HZ)

#define MAHONY_KP  1.0f
#define MAHONY_KI  1.0f

static bool g_imu_ready, g_mag_ready, g_i2c_installed;
static i2c_port_t g_i2c_port;
static uint8_t g_mpu_addr;
static float g_bias_ax, g_bias_ay, g_bias_az;
static float g_bias_gx, g_bias_gy, g_bias_gz;
static float g_mag_hard_iron[3];
static TaskHandle_t g_imu_task;
static volatile bool g_task_run;
static volatile float g_cached_roll, g_cached_pitch, g_cached_yaw;
static imu_raw_data_t g_cached_raw;

static void imu_task_main(void *pvParam);
static void imu_load_cal(void);
static void imu_save_cal(void);

static esp_err_t w(uint8_t a, uint8_t r, uint8_t v) {
    uint8_t b[2]={r,v};
    return i2c_master_write_to_device(g_i2c_port,a,b,2,pdMS_TO_TICKS(10));
}
static esp_err_t r(uint8_t a, uint8_t reg, uint8_t *d, size_t n) {
    return i2c_master_write_read_device(g_i2c_port,a,&reg,1,d,n,pdMS_TO_TICKS(10));
}

/* SLV0 写/读 AK8963 — 先停再配 */
static void slv0_write(uint8_t reg, uint8_t val) {
    w(g_mpu_addr, MPU_SLV0_CTRL, 0x00); vTaskDelay(pdMS_TO_TICKS(2));
    w(g_mpu_addr, MPU_SLV0_ADDR, AK8963_ADDR);
    w(g_mpu_addr, MPU_SLV0_REG, reg);
    w(g_mpu_addr, MPU_SLV0_DO, val);
    w(g_mpu_addr, MPU_SLV0_CTRL, SLV0_EN|1);
    vTaskDelay(pdMS_TO_TICKS(10));
}
static void slv0_read(uint8_t reg, uint8_t n, uint8_t *d) {
    w(g_mpu_addr, MPU_SLV0_CTRL, 0x00); vTaskDelay(pdMS_TO_TICKS(2));
    w(g_mpu_addr, MPU_SLV0_ADDR, AK8963_ADDR|0x80);
    w(g_mpu_addr, MPU_SLV0_REG, reg);
    w(g_mpu_addr, MPU_SLV0_CTRL, SLV0_EN|n);
    vTaskDelay(pdMS_TO_TICKS(10));
    r(g_mpu_addr, MPU_EXT_SENS_DATA, d, n);
}
/* SLV0 后台持续读 */
static void slv0_bg(void) {
    w(g_mpu_addr, MPU_SLV0_CTRL, 0x00); vTaskDelay(pdMS_TO_TICKS(2));
    w(g_mpu_addr, MPU_SLV0_ADDR, AK8963_ADDR|0x80);
    w(g_mpu_addr, MPU_SLV0_REG, AK8963_ST1);
    w(g_mpu_addr, MPU_SLV0_CTRL, SLV0_EN|8);
    vTaskDelay(pdMS_TO_TICKS(5));
}

static esp_err_t ak8963_init(void) {
    uint8_t who;
    slv0_read(AK8963_WHO_AM_I, 1, &who);
    mp_printf(&mp_plat_print, "[imu] AK8963 WHOAMI:0x%02X\n", who);
    if (who != 0x48) return ESP_FAIL;
    slv0_write(AK8963_CNTL2, 0x01); vTaskDelay(pdMS_TO_TICKS(10));
    slv0_write(AK8963_CNTL1, 0x16); vTaskDelay(pdMS_TO_TICKS(10));
    slv0_bg();  // ← 关键: SLV0 后台读 8 字节
    g_mag_ready = true;
    mp_printf(&mp_plat_print, "[imu] AK8963 ready\n");
    return ESP_OK;
}

void imu_init(uint8_t port, uint8_t sda, uint8_t scl, uint8_t addr) {
    g_i2c_port = (i2c_port_t)port;
    (void)addr;
    if (!g_i2c_installed) {
        i2c_config_t c = {.mode=I2C_MODE_MASTER,.sda_io_num=sda,.scl_io_num=scl,
                          .sda_pullup_en=GPIO_PULLUP_ENABLE,.scl_pullup_en=GPIO_PULLUP_ENABLE,
                          .master.clk_speed=100000};
        ESP_ERROR_CHECK(i2c_param_config(g_i2c_port,&c));
        ESP_ERROR_CHECK(i2c_driver_install(g_i2c_port,c.mode,0,0,0));
        g_i2c_installed = true;
    }
    for (uint8_t a=0x68; a<=0x69; a++) {
        uint8_t ww; if (r(a,MPU_WHO_AM_I,&ww,1)==ESP_OK&&(ww==0x71||ww==0x73))
            {g_mpu_addr=a; mp_printf(&mp_plat_print,"[imu] MPU9250@0x%02X\n",a); break;}
    }
    if (!g_mpu_addr) {mp_printf(&mp_plat_print,"[imu] NOT FOUND\n"); return;}

    w(g_mpu_addr, MPU_PWR_MGMT_1, CLKSEL_PLL); vTaskDelay(pdMS_TO_TICKS(100));
    w(g_mpu_addr, MPU_USER_CTRL, I2C_MST_EN); vTaskDelay(pdMS_TO_TICKS(5));
    w(g_mpu_addr, MPU_USER_CTRL, I2C_MST_EN|I2C_MST_RST); vTaskDelay(pdMS_TO_TICKS(5));
    w(g_mpu_addr, MPU_INT_PIN_CFG, 0x00);
    w(g_mpu_addr, MPU_I2C_MST_CTRL, 0x04);
    w(g_mpu_addr, MPU_CONFIG, 0x03);
    w(g_mpu_addr, MPU_ACCEL_CONFIG2, 0x03);
    w(g_mpu_addr, MPU_GYRO_CONFIG, 0x08);
    w(g_mpu_addr, MPU_ACCEL_CONFIG, 0x10);
    ak8963_init();
    imu_load_cal();
    g_task_run = true;
    xTaskCreatePinnedToCore(imu_task_main,"imu_ahrs",IMU_TASK_STACK,NULL,IMU_TASK_PRIO,&g_imu_task,1);
    g_imu_ready = true;
    mp_printf(&mp_plat_print,"[imu] ready mag=%s\n",g_mag_ready?"OK":"NO");
}

static bool read_all(imu_raw_data_t *d) {
    uint8_t b[14];
    if (r(g_mpu_addr, MPU_ACCEL_XOUT_H, b, 14) != ESP_OK) return false;
    int16_t ax=(b[0]<<8)|b[1], ay=(b[2]<<8)|b[3], az=(b[4]<<8)|b[5];
    int16_t gt=(b[6]<<8)|b[7];
    int16_t gx=(b[8]<<8)|b[9], gy=(b[10]<<8)|b[11], gz=(b[12]<<8)|b[13];
    d->accel_x=(ax/ACCEL_SCALE)*G_MPS2-g_bias_ax;
    d->accel_y=(ay/ACCEL_SCALE)*G_MPS2-g_bias_ay;
    d->accel_z=(az/ACCEL_SCALE)*G_MPS2-g_bias_az;
    d->gyro_x=(gx/GYRO_SCALE)*DEG2RAD-g_bias_gx;
    d->gyro_y=(gy/GYRO_SCALE)*DEG2RAD-g_bias_gy;
    d->gyro_z=(gz/GYRO_SCALE)*DEG2RAD-g_bias_gz;
    d->temp_c=gt/TEMP_SCALE+21.0f;
    // ② Mag: 直接读 EXT_SENS_DATA (SLV0 后台持续刷新)
    d->mag_x=d->mag_y=d->mag_z=0.0f;
    if (g_mag_ready) {
        uint8_t mbuf[8];
        if (r(g_mpu_addr, MPU_EXT_SENS_DATA, mbuf, 8)==ESP_OK
            && !(mbuf[7]&AK8963_HOFL)) {
            int16_t mx=(mbuf[2]<<8)|mbuf[1], my=(mbuf[4]<<8)|mbuf[3], mz=(mbuf[6]<<8)|mbuf[5];
            d->mag_x=mx*MAG_SCALE-g_mag_hard_iron[0];
            d->mag_y=my*MAG_SCALE-g_mag_hard_iron[1];
            d->mag_z=mz*MAG_SCALE-g_mag_hard_iron[2];
        }
    }
    // 轴重映射: 前=-Y, 右=-X, 上=Z
    float af=-d->accel_y, ar=-d->accel_x, au=d->accel_z;
    float gf=-d->gyro_y, gr=-d->gyro_x, gu=d->gyro_z;
    float mf=-d->mag_y, mr=-d->mag_x, mu=d->mag_z;
    d->accel_x=af; d->accel_y=ar; d->accel_z=au;
    d->gyro_x=gf; d->gyro_y=gr; d->gyro_z=gu;
    d->mag_x=mf; d->mag_y=mr; d->mag_z=mu;
    return true;
}

static float q0=1,q1,q2,q3,ifx,ify,ifz;
static void mahony(float gx,float gy,float gz,float ax,float ay,float az,float mx,float my,float mz,float dt) {
    if (dt<=0) return;
    float n=sqrtf(ax*ax+ay*ay+az*az);
    if (n<0.01f){ax=0;ay=0;az=1;}else{ax/=n;ay/=n;az/=n;}
    n=sqrtf(mx*mx+my*my+mz*mz);
    if (n<0.3f){mx=my=mz=0;}else{mx/=n;my/=n;mz/=n;}
    float hvx=q1*q3-q0*q2, hvy=q0*q1+q2*q3, hvz=q0*q0-0.5f+q3*q3;
    float hex=ay*hvz-az*hvy, hey=az*hvx-ax*hvz, hez=ax*hvy-ay*hvx;
    if (mx||my||mz) {
        float hx=mx*(q0*q0+q1*q1-0.5f)+my*(q1*q2-q0*q3)+mz*(q1*q3+q0*q2);
        float hy=mx*(q1*q2+q0*q3)+my*(q0*q0+q2*q2-0.5f)+mz*(q2*q3-q0*q1);
        float bx=sqrtf(hx*hx+hy*hy);
        float hwx=bx*(0.5f-q2*q2-q3*q3), hwy=bx*(q1*q2-q0*q3), hwz=bx*(q1*q3+q0*q2);
        hex+=my*hwz-mz*hwy; hey+=mz*hwx-mx*hwz; hez+=mx*hwy-my*hwx;
    }
    ifx+=MAHONY_KI*hex*dt; ify+=MAHONY_KI*hey*dt; ifz+=MAHONY_KI*hez*dt;
    gx+=MAHONY_KP*hex+ifx; gy+=MAHONY_KP*hey+ify; gz+=MAHONY_KP*hez+ifz;
    float qa=q0,qb=q1,qc=q2,qd=q3;
    q0+=(-qb*gx-qc*gy-qd*gz)*0.5f*dt; q1+=(qa*gx+qd*gy-qc*gz)*0.5f*dt;
    q2+=(qd*gx+qa*gy+qb*gz)*0.5f*dt; q3+=(-qc*gx+qb*gy+qa*gz)*0.5f*dt;
    n=sqrtf(q0*q0+q1*q1+q2*q2+q3*q3);
    if (n>0.0001f){q0/=n;q1/=n;q2/=n;q3/=n;}
}

static void imu_task_main(void *pv) {
    TickType_t lw=xTaskGetTickCount();
    const TickType_t p=pdMS_TO_TICKS(IMU_TASK_PERIOD);
    int64_t lu=0;
    while (g_task_run) {
        imu_raw_data_t d={0};
        if (read_all(&d)) {
            int64_t nw=esp_timer_get_time();
            float dt=(lu==0)?(1.0f/IMU_TASK_HZ):(nw-lu)*1e-6f; lu=nw;
            mahony(d.gyro_x,d.gyro_y,d.gyro_z,d.accel_x,d.accel_y,d.accel_z,d.mag_x,d.mag_y,d.mag_z,dt);
            float s1=q1*q1,s2=q2*q2,s3=q3*q3;
            g_cached_roll=atan2f(2*(q0*q1+q2*q3),1-2*(s1+s2))*57.29578f;
            g_cached_pitch=asinf(2*(q0*q2-q3*q1))*57.29578f;
            g_cached_yaw=atan2f(2*(q0*q3+q1*q2),1-2*(s2+s3))*57.29578f;
            memcpy((void*)&g_cached_raw,&d,sizeof(d));
        }
        vTaskDelayUntil(&lw,p);
    }
    vTaskDelete(NULL);
}

void imu_read_raw(imu_raw_data_t *d) {if(g_imu_ready&&d)memcpy(d,(void*)&g_cached_raw,sizeof(*d));}
void imu_read_angles(imu_angles_t *a) {
    if (!g_imu_ready||!a) return;
    a->roll=g_cached_roll; a->pitch=g_cached_pitch; a->yaw=g_cached_yaw;
}

static void imu_save_cal(void) {
    nvs_handle_t h; if (nvs_open(IMU_CAL_NS,NVS_READWRITE,&h)!=ESP_OK) return;
    nvs_set_i32(h,"gbx",(int32_t)(g_bias_gx*1000)); nvs_set_i32(h,"gby",(int32_t)(g_bias_gy*1000));
    nvs_set_i32(h,"gbz",(int32_t)(g_bias_gz*1000)); nvs_set_i32(h,"abx",(int32_t)(g_bias_ax*1000));
    nvs_set_i32(h,"aby",(int32_t)(g_bias_ay*1000)); nvs_set_i32(h,"abz",(int32_t)(g_bias_az*1000));
    nvs_set_i32(h,"mhx",(int32_t)(g_mag_hard_iron[0]*100)); nvs_set_i32(h,"mhy",(int32_t)(g_mag_hard_iron[1]*100));
    nvs_set_i32(h,"mhz",(int32_t)(g_mag_hard_iron[2]*100)); nvs_commit(h); nvs_close(h);
}
static void imu_load_cal(void) {
    nvs_handle_t h; if (nvs_open(IMU_CAL_NS,NVS_READONLY,&h)!=ESP_OK) return; int32_t v;
    if (nvs_get_i32(h,"gbx",&v)==ESP_OK) g_bias_gx=v/1000.0f;
    if (nvs_get_i32(h,"gby",&v)==ESP_OK) g_bias_gy=v/1000.0f;
    if (nvs_get_i32(h,"gbz",&v)==ESP_OK) g_bias_gz=v/1000.0f;
    if (nvs_get_i32(h,"abx",&v)==ESP_OK) g_bias_ax=v/1000.0f;
    if (nvs_get_i32(h,"aby",&v)==ESP_OK) g_bias_ay=v/1000.0f;
    if (nvs_get_i32(h,"abz",&v)==ESP_OK) g_bias_az=v/1000.0f;
    if (nvs_get_i32(h,"mhx",&v)==ESP_OK) g_mag_hard_iron[0]=v/100.0f;
    if (nvs_get_i32(h,"mhy",&v)==ESP_OK) g_mag_hard_iron[1]=v/100.0f;
    if (nvs_get_i32(h,"mhz",&v)==ESP_OK) g_mag_hard_iron[2]=v/100.0f;
    nvs_close(h);
}

void imu_calibrate(int n) {
    if (!g_imu_ready||n<10) return;
    if (g_imu_task){g_task_run=false; vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD*2));}
    mp_printf(&mp_plat_print,"Gyro cal %d samples...\n",n);
    float sa=0,sb=0,sc=0,sx=0,sy=0,sz=0;
    for (int i=0;i<n;i++) {
        uint8_t b[14];
        if (r(g_mpu_addr,MPU_ACCEL_XOUT_H,b,14)!=ESP_OK) continue;
        sa+=(b[0]<<8)|b[1]; sb+=(b[2]<<8)|b[3]; sc+=(b[4]<<8)|b[5];
        sx+=(b[8]<<8)|b[9]; sy+=(b[10]<<8)|b[11]; sz+=(b[12]<<8)|b[13];
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    float nn=(float)n;
    g_bias_ax=0; g_bias_ay=0; g_bias_az=(sc/nn/ACCEL_SCALE*G_MPS2)-G_MPS2;
    g_bias_gx=sx/nn/GYRO_SCALE*DEG2RAD; g_bias_gy=sy/nn/GYRO_SCALE*DEG2RAD; g_bias_gz=sz/nn/GYRO_SCALE*DEG2RAD;
    q0=1;q1=q2=q3=0; ifx=ify=ifz=0;
    g_task_run=true;
    xTaskCreatePinnedToCore(imu_task_main,"imu_ahrs",IMU_TASK_STACK,NULL,IMU_TASK_PRIO,&g_imu_task,1);
    imu_save_cal();
    mp_printf(&mp_plat_print,"Gyro cal done\n");
}

STATIC mp_obj_t mi_init(size_t n,const mp_obj_t *a){imu_init(mp_obj_get_int(a[0]),mp_obj_get_int(a[1]),mp_obj_get_int(a[2]),mp_obj_get_int(a[3]));return mp_const_none;}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mi_init_o,4,4,mi_init);

STATIC mp_obj_t mi_raw(void){imu_raw_data_t d;memset(&d,0,sizeof(d));imu_read_raw(&d);
    mp_obj_t aa[3]={mp_obj_new_float(d.accel_x),mp_obj_new_float(d.accel_y),mp_obj_new_float(d.accel_z)};
    mp_obj_t gg[3]={mp_obj_new_float(d.gyro_x),mp_obj_new_float(d.gyro_y),mp_obj_new_float(d.gyro_z)};
    mp_obj_t mm[3]={mp_obj_new_float(d.mag_x),mp_obj_new_float(d.mag_y),mp_obj_new_float(d.mag_z)};
    mp_obj_t t[4]={mp_obj_new_tuple(3,aa),mp_obj_new_tuple(3,gg),mp_obj_new_tuple(3,mm),mp_obj_new_float(d.temp_c)};
    return mp_obj_new_tuple(4,t);}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mi_raw_o,mi_raw);

STATIC mp_obj_t mi_ang(void){imu_angles_t a={0};imu_read_angles(&a);
    mp_obj_t t[3]={mp_obj_new_float(a.roll),mp_obj_new_float(a.pitch),mp_obj_new_float(a.yaw)};
    return mp_obj_new_tuple(3,t);}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mi_ang_o,mi_ang);

STATIC mp_obj_t mi_cal(mp_obj_t s){imu_calibrate(mp_obj_get_int(s));return mp_const_none;}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mi_cal_o,mi_cal);

STATIC const mp_rom_map_elem_t table[]={
    {MP_ROM_QSTR(MP_QSTR___name__),MP_ROM_QSTR(MP_QSTR_bpuppy_imu)},
    {MP_ROM_QSTR(MP_QSTR_init),MP_ROM_PTR(&mi_init_o)},
    {MP_ROM_QSTR(MP_QSTR_read_raw),MP_ROM_PTR(&mi_raw_o)},
    {MP_ROM_QSTR(MP_QSTR_read_angles),MP_ROM_PTR(&mi_ang_o)},
    {MP_ROM_QSTR(MP_QSTR_calibrate),MP_ROM_PTR(&mi_cal_o)},
};
STATIC MP_DEFINE_CONST_DICT(glob,table);
const mp_obj_module_t bpuppy_imu_module={.base={&mp_type_module},.globals=(mp_obj_dict_t*)&glob};
MP_REGISTER_MODULE(MP_QSTR_bpuppy_imu,bpuppy_imu_module);
