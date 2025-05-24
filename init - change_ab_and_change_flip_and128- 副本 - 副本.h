#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "bit.h"

#include <vector>



/* 模块端口寄存器 */
#define CTR_REG 0
#define CACHE_ADDR_REG 1
#define CACHE_SIZE_REG 2
#define LINE_NUM_REG 3
#define INTERVAL_TIME_REG 4
#define CHOSE 5
#define BLACK_OFFSET 6

/* 命令 */
#define LINE_DATA_READY 0x00000100
/* CIS工作模式 */
#define MODE1 0 
#define MODE2 1 
#define MODE3 2 
/* 扫描尺寸参数 */
#define SCAN_LINE_NUM 3507             //////////////// 
#define PIXEL_NUM_PER_LINE 2592
/* 缓存 */
#define CACHE_SIZE PIXEL_NUM_PER_LINE * 3 * 50   

//#define SER_IP_ADDR "172.31.225.181"
#define SER_PORT 8000
#define DATA_SIZE SCAN_LINE_NUM * PIXEL_NUM_PER_LINE * 3 
// ioctl命令
#define CMD_ALLOC_MEM   (_IO('a', 1))         
#define CMD_SET_DPI     (_IO('a', 2))        /* 设置dpi模式 */

struct trandata {
    unsigned int number;
    unsigned int data;
};

struct trandata buf;
static void* base = NULL;
static bool is_flip = true;
static bool change_scan = false;
std::vector<std::vector<double>>* black_level;
std::vector<std::vector<double>>* white1_weight;
//std::vector<std::vector<double>>* white_avg;
//std::vector<std::vector<double>>* black_avg;
//unsigned char * s_addr = NULL;

void write_reg(int fd, unsigned int number, unsigned int data)
{
    buf.number = number;
    buf.data = data;
    int ret = write(fd, &buf, sizeof(buf));
    if (0 > ret) {
        printf("Write reg %d failed!\r\n", number);
        close(fd);
        exit(1);
    }
}

void read_reg(int fd, unsigned int number, unsigned int* data)
{
    buf.number = number;
    buf.data = 0;
    read(fd, &buf, sizeof(buf));
    *data = buf.data;
}

void set_scan_mode(int fd, unsigned char mode)
{
    int ret = 0;
    ret = ioctl(fd, CMD_SET_DPI, mode);
    if (ret) {
        if (errno == EINVAL) {
            printf("Scan Mode Parameter Error!!\r\n");
            close(fd);
            exit(1);
        }
    }
}

void img_flip(void)
{
    if (is_flip)
        is_flip = false;
    else
        is_flip = true;
}

void change_scanner(int change)
{
    if (change == 0)
        change_scan = false;
    else if (change == 1)
        change_scan = true;
}

void write_bmp(char* bmpfile, unsigned char* addr)
{
    unsigned int width = PIXEL_NUM_PER_LINE;
    unsigned char bit_count_per_pixel = 24;
    unsigned int line_index = SCAN_LINE_NUM;
    FILE* bmp_fp = create_bmp_file(bit_count_per_pixel, PIXEL_NUM_PER_LINE, SCAN_LINE_NUM, bmpfile);
    while (line_index) {
        unsigned char* pdata = NULL;
        unsigned int cnt = 300;

        if (line_index < 300)
            cnt = line_index;
        pdata = (unsigned char*)malloc(width * cnt * 3);
        // 数据分多次写入bmp文件，每次写入cnt行数据
        for (int i = 0; i < cnt; i++) {
            unsigned char* line_data_addr = addr + ((line_index - 1 - i) * width) * 3;
            // bmp的存储格式是BGR，而预留内存中的存储顺序也是BGR
            for (int j = 0; j < width; j++) {
                int offset = 0;
                if (is_flip)
                    offset = (width - 1 - j) * 3;   // 如果cis反着装，则数据要从后往前读
                else
                    offset = j * 3;
                pdata[(i * width + j) * 3 + 0] = line_data_addr[offset];
                pdata[(i * width + j) * 3 + 1] = line_data_addr[offset + 1];
                pdata[(i * width + j) * 3 + 2] = line_data_addr[offset + 2];
            }
        }

        write_data_to_bmp(bmp_fp, (unsigned char*)pdata, width, cnt, bit_count_per_pixel);
        fflush(bmp_fp); // 强制刷新缓冲区到操作系统
        fsync(fileno(bmp_fp)); // 强制从操作系统缓存写入磁盘（Linux）
        free(pdata);
        line_index -= cnt;
    }
}

bool update_bmpFile(const char* filename) {
    bmpInfo my_bmp;
    GetBmpInfo(&my_bmp, filename);

    U8* pdata = my_bmp.data;
    U8 BytePerPix = (my_bmp.bitCountPerPix) >> 3;
    U32 pitch = (my_bmp.col) * BytePerPix;

    int w, h;
    for (w = 0; w < my_bmp.col; w++) {
        for (h = my_bmp.row - 1; h >= 0; h--) {
            for (int i = 0; i < 3; i++) {
                int current_w = w;
                if (is_flip==false) {
                    // 如果需要翻转，则使用翻转后的列索引
                    current_w = my_bmp.col - 1 - w;
                }
                //float raw_value = pdata[h * pitch + w * BytePerPix + i];
                //// float corrected_value = raw_value - 120;       //  (*black_level)[w][i]   把这个改为120

                // //// 应用白参考校正
                // //if (corrected_value < 0) {
                // //    corrected_value = 0;
                // //}
                //float corrected_value = (raw_value - 120) * 255 / ((*white1_weight)[w][i] - 120);
                //// 应用黑参考校正
                float raw_value = pdata[h * pitch + current_w * BytePerPix + i];
                float corrected_value = (raw_value - 120) * 255 / ((*white1_weight)[w][i] - 120);

                
                //corrected_value = corrected_value * 255 / ((*white1_weight)[w][i] - (*black_level)[w][i]);

                // 边界检查
                if (corrected_value > 255) {
                    pdata[h * pitch + current_w * BytePerPix + i] = 255;
                }
                else if (corrected_value < 0) {
                    pdata[h * pitch + current_w * BytePerPix + i] = 0;
                }
                else {
                    pdata[h * pitch + current_w * BytePerPix + i] = (U8)corrected_value;
                }
            }
        }
    }

    // 保存校正后的图像
    const char* my_name = "/home/root/cis_app/res.bmp";
    GenBmpFile(my_bmp.data, my_bmp.bitCountPerPix, my_bmp.col, my_bmp.row, my_name);
    return true;
}

void get_img(int fd, bool is_white_img, bool is_black_img)
{
    unsigned char* s_addr = (unsigned char*)malloc(DATA_SIZE);
    unsigned char* p = s_addr;
    size_t s = DATA_SIZE;
    unsigned int ctr_data;
    // 设置启动位
    read_reg(fd, CTR_REG, &ctr_data);
    write_reg(fd, CTR_REG, ctr_data | 4);
    //read_reg(fd, CTR_REG, &ctr_data);
    //printf("line num reg = %lu\r\n",ctr_data);
    while (1) {
        base = mmap(NULL, CACHE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if ((void*)-1 == base) {
            if (errno == ENOBUFS) {
                printf("Transmission completed.\r\n");
                break;
            }
            else if (errno == ENOMEM) {
                //printf("continue.\r\n");
                continue;
            }
        }
        size_t data_len = (s >= CACHE_SIZE) ? CACHE_SIZE : s;
        memcpy(p, base, data_len);
        munmap(base, CACHE_SIZE);
        p = p + data_len;
        s -= data_len;
    }
    if (is_white_img && is_black_img == 0) {
        //为false就写入a的照片，否则写入b的，两者的照片名字不一样
        if (change_scan == false)
            write_bmp("/home/root/cis_app/white.bmp", s_addr);
        else
            write_bmp("/home/root/cis_app/white_b.bmp", s_addr);

    }
    else if (is_black_img && is_white_img == 0) {
        if (change_scan == false)
            write_bmp("/home/root/cis_app/black.bmp", s_addr);
        else
            write_bmp("/home/root/cis_app/black_b.bmp", s_addr);
    }
    else
        write_bmp("/home/root/cis_app/origin.bmp", s_addr);
    free(s_addr);
}

void start_scan(int fd)
{
    unsigned char* s_addr = (unsigned char*)malloc(DATA_SIZE);
    unsigned char* p = s_addr;
    size_t s = DATA_SIZE;
    unsigned int ctr_data;
    // 设置启动位
    read_reg(fd, CTR_REG, &ctr_data);
    write_reg(fd, CTR_REG, ctr_data | 4);
    //read_reg(fd, CTR_REG, &ctr_data);
    //printf("line num reg = %lu\r\n",ctr_data);
    while (1) {
        base = mmap(NULL, CACHE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if ((void*)-1 == base) {
            if (errno == ENOBUFS) {
                printf("Transmission completed.\r\n");
                break;
            }
            else if (errno == ENOMEM) {
                //printf("continue.\r\n");
                continue;
            }
        }
        size_t data_len = (s >= CACHE_SIZE) ? CACHE_SIZE : s;
        memcpy(p, base, data_len);
        munmap(base, CACHE_SIZE);
        p = p + data_len;
        s -= data_len;
    }
    //unsigned char* _addr = (unsigned char *)malloc(DATA_SIZE);
    //memset(_addr, '150', DATA_SIZE);
    write_bmp("/home/root/cis_app/origin.bmp", s_addr);
    update_bmpFile("/home/root/cis_app/origin.bmp");
    free(s_addr);
}

void set_interval_time(int fd, unsigned int t)
{
    write_reg(fd, INTERVAL_TIME_REG, t);
    unsigned int ctr_data;
    read_reg(fd, INTERVAL_TIME_REG, &ctr_data);
    printf("interval time = %d\n", ctr_data);
    fflush(stdout);
}

/* 初始化权重 */
std::vector<std::vector<double>> init_weight(const char* filename) {
    bmpInfo* info = new bmpInfo;
    GetBmpInfo(info, filename);

    std::vector<std::vector<double>> white_weight(info->col, std::vector<double>(3, 1.0));
    std::vector<std::vector<double>> white_avg(info->col, std::vector<double>(3, 0.0));     //全都初始化为0
    U8* pdata = info->data;
    U8 BytePerPix = (info->bitCountPerPix) >> 3;
    U32 pitch = (info->col) * BytePerPix;
    int w, h;

    for (w = 0; w < info->col; w++) {
        std::vector<long long> sum(3, 0);
        for (h = info->row - 1; h >= 0; h--) {
            sum[0] += pdata[h * pitch + w * BytePerPix + 0];
            sum[1] += pdata[h * pitch + w * BytePerPix + 1];
            sum[2] += pdata[h * pitch + w * BytePerPix + 2];
        }

        for (int i = 0; i < 3; i++) {
            // int avg = sum[i] / info->row;  //记得转成浮点数
            white_avg[w][i] = sum[i] / info->row;       //我的avg实际上是对列取平均的，所以列会比较均匀，行没有均匀
            /*if (avg != 255) {
                white_weight[w][i] = 255.0 / avg;
            }*/
        }
        sum.clear();
    }
    FreeBmpData(pdata);
    delete info;

    //return white_weight;
    return white_avg;
}

std::vector<std::vector<double>> init_weight_black(const char* filename) {
    bmpInfo* info = new bmpInfo;
    GetBmpInfo(info, filename);

    std::vector<std::vector<double>> black_weight(info->col, std::vector<double>(3, 0.0));
    std::vector<std::vector<double>> black_avg(info->col, std::vector<double>(3, 0.0));
    U8* pdata = info->data;
    U8 BytePerPix = (info->bitCountPerPix) >> 3;
    U32 pitch = (info->col) * BytePerPix;
    int w, h;

    for (w = 0; w < info->col; w++) {
        std::vector<long long> sum(3, 0);
        for (h = info->row - 1; h >= 0; h--) {
            sum[0] += pdata[h * pitch + w * BytePerPix + 0];
            sum[1] += pdata[h * pitch + w * BytePerPix + 1];
            sum[2] += pdata[h * pitch + w * BytePerPix + 2];
        }

        for (int i = 0; i < 3; i++) {
            int avg = sum[i] / info->row;
            //black_weight[w][i] = avg;
            black_avg[w][i] = avg;
        }
        sum.clear();
    }
    FreeBmpData(pdata);
    delete info;

    //return black_weight;
    return black_avg;
}

void update_weight(void) {

    if (change_scan == false) {
        std::vector<std::vector<double>> white_avg = init_weight("/home/root/cis_app/white.bmp");
        // 初始化黑参考权重
        std::vector<std::vector<double>> black_avg = init_weight_black("/home/root/cis_app/black.bmp");

        // 结合白参考和黑参考权重
        white1_weight = new std::vector<std::vector<double>>(white_avg.size(), std::vector<double>(3, 1.0));
        black_level = new std::vector<std::vector<double>>(white_avg.size(), std::vector<double>(3, 0.0));

        for (int w = 0; w < white_avg.size(); w++) {
            for (int i = 0; i < 3; i++) {
                (*white1_weight)[w][i] = white_avg[w][i];
                (*black_level)[w][i] = black_avg[w][i];
            }
        }
    }

    else {
        std::vector<std::vector<double>> white_avg = init_weight("/home/root/cis_app/white_b.bmp");
        // 初始化黑参考权重
        std::vector<std::vector<double>> black_avg = init_weight_black("/home/root/cis_app/black_b.bmp");
        // 结合白参考和黑参考权重
        white1_weight = new std::vector<std::vector<double>>(white_avg.size(), std::vector<double>(3, 1.0));
        black_level = new std::vector<std::vector<double>>(white_avg.size(), std::vector<double>(3, 0.0));

        for (int w = 0; w < white_avg.size(); w++) {
            for (int i = 0; i < 3; i++) {
                (*white1_weight)[w][i] = white_avg[w][i];
                (*black_level)[w][i] = black_avg[w][i];
            }
        }
    }

}

// 定义一个新的结构体用于返回多个值
typedef struct {
    unsigned char high8; // 无符号8位整数
    unsigned char mid8;  // 无符号8位整数
    unsigned char low8;  // 无符号8位整数
} BlackOffsetValues;

BlackOffsetValues read_black_offset(int fd) {
    unsigned int reg_value;
    BlackOffsetValues result = { 0, 0, 0 }; // 初始化为0

    // 读取 BLACK_OFFSET 寄存器的值
    read_reg(fd, BLACK_OFFSET, &reg_value);

    // 提取第16到23位的值并判断
    unsigned char high8 = (reg_value >> 16) & 0xFF;
    if (high8 > 0) {
        result.high8 = high8;
    }

    // 提取第8到15位的值并判断
    unsigned char mid8 = (reg_value >> 8) & 0xFF;
    if (mid8 > 0) {
        result.mid8 = mid8;
    }

    // 提取第0到7位的值并判断
    unsigned char low8 = reg_value & 0xFF;
    if (low8 > 0) {
        result.low8 = low8;
    }
    // 以十进制形式打印 high8, mid8, low8
    printf("high8: %d\n", result.high8);
    printf("mid8: %d\n", result.mid8);
    printf("low8: %d\n", result.low8);
    return result;
}


void chose_scanner(int fd, int choose) {
    unsigned int chose_data = 0;
    // 读取 CHOSE 寄存器的当前值
    read_reg(fd, CHOSE, &chose_data);

    if (choose == 1) {
        chose_data |= (1 << 31); // 最高位
        chose_data |= 1; // 最低位
    }
    else if (choose == 0) {
        chose_data &= ~(1 << 31); // 最高位
        chose_data &= ~1; // 最低位
    }

    // 写回 CHOSE 寄存器
    write_reg(fd, CHOSE, chose_data);

    //read_reg(fd, CHOSE, &chose_data);


}


void dpi_chose(int fd, int choose) {

    unsigned int ctr_data;
    // 读取 CTR_REG寄存器的当前值
    read_reg(fd, CTR_REG, &ctr_data);

    //300dpi模式
    if (choose == 0) {
        // 写回 CHOSE 寄存器
        write_reg(fd, CTR_REG, ctr_data & ~3);
    }
    //600dpi模式
    else if (choose == 1) {
        write_reg(fd, CTR_REG, (ctr_data & ~3) | 1);
    }
    //1200dpi模式
    else if (choose == 2) {
        write_reg(fd, CTR_REG, (ctr_data & ~3) | 2);
    }

}




int init_device(unsigned long* ctr_data)
{
    int fd;
    fd = open("/dev/scanner", O_RDWR);
    if (0 > fd) {
        printf("file /dev/scanner open failed!\r\n");
        return -1;
    }
    //一开始默认是扫描仪a的即可，取这两个的参数
    init_weight("/home/root/cis_app/white.bmp");
    init_weight_black("/home/root/cis_app/black.bmp");
    update_weight();

    /* 设置缓存大小 */
    write_reg(fd, CACHE_SIZE_REG, CACHE_SIZE);

    /* 设置要扫描的行数 */
    write_reg(fd, LINE_NUM_REG, SCAN_LINE_NUM);
    write_reg(fd, INTERVAL_TIME_REG, 2);
    /* 设置扫描模式 */
    set_scan_mode(fd, MODE1);

    //write_reg(fd, CTR_REG, 5);
    //read_reg(fd, CACHE_SIZE_REG, ctr_data);
    //printf("cache size reg = %lu\r\n",ctr_data);

    //read_reg(fd, LINE_NUM_REG, ctr_data);
    //printf("line num reg = %lu\r\n",ctr_data);

    //read_reg(fd, INTERVAL_TIME_REG, ctr_data);
    //printf("interval time reg = %lu\r\n",ctr_data);

    //read_reg(fd, CTR_REG, ctr_data);
    //printf("ctl reg = %lu\r\n",ctr_data);
    ioctl(fd, CMD_ALLOC_MEM, 18);
    return fd;
}


/////////////////////////////////////////////////////////////////////////////////////////////
//// 
//// 
////    这些代码是用来把像素的值稍加处理，去平均值，保存到文本里面的代码
//// 
//// 
//// ///////////////////////////////////////////////////////////////////////////////////////
//// 定义一个函数来处理black_avg并生成black_new_avg
//std::vector<std::vector<double>> downsample_black_avg(const std::vector<std::vector<double>>& black_avg) {
//    int total_rows = black_avg.size(); // 总行数
//    int cols = black_avg[0].size(); // 列数（应该是3）
//    int new_rows = (total_rows + 9) / 10; // 计算新的行数，向上取整
//
//    // 初始化black_new_avg
//    std::vector<std::vector<double>> black_new_avg(new_rows, std::vector<double>(cols, 0.0));
//
//    // 处理前3500行（取10的倍数行）
//    for (int i = 0; i < 3500; i += 10) {
//        for (int j = 0; j < cols; ++j) {
//            black_new_avg[i / 10][j] = black_avg[i][j];
//        }
//    }
//
//    // 处理最后7行（取平均值）
//    if (total_rows > 3500) {
//        int remaining_rows = total_rows - 3500; // 剩余的行数
//        std::vector<double> sum(cols, 0.0); // 用于累加剩余行的值
//        for (int i = 3500; i < total_rows; ++i) {
//            for (int j = 0; j < cols; ++j) {
//                sum[j] += black_avg[i][j];
//            }
//        }
//        // 计算平均值
//        for (int j = 0; j < cols; ++j) {
//            black_new_avg[350][j] = sum[j] / remaining_rows;
//        }
//    }
//
//    return black_new_avg;
//}
//
//// 计算并输出black_new_avg的平均值、最大值和最小值
//void compute_statistics(const std::vector<std::vector<double>>& black_new_avg, std::ofstream& file) {
//    int rows = black_new_avg.size();
//    int cols = black_new_avg[0].size();
//
//    // 初始化统计变量
//    std::vector<double> avg(cols, 0.0);
//    std::vector<double> max_val(cols, -std::numeric_limits<double>::infinity());
//    std::vector<double> min_val(cols, std::numeric_limits<double>::infinity());
//
//    // 计算总和、最大值和最小值
//    for (int i = 0; i < rows; ++i) {
//        for (int j = 0; j < cols; ++j) {
//            avg[j] += black_new_avg[i][j];
//            if (black_new_avg[i][j] > max_val[j]) {
//                max_val[j] = black_new_avg[i][j];
//            }
//            if (black_new_avg[i][j] < min_val[j]) {
//                min_val[j] = black_new_avg[i][j];
//            }
//        }
//    }
//
//    // 计算平均值
//    for (int j = 0; j < cols; ++j) {
//        avg[j] /= rows;
//    }
//
//    // 输出结果到文件
//    file << "Statistics for black_new_avg:" << std::endl;
//    file << "Average values: R = " << avg[0] << ", G = " << avg[1] << ", B = " << avg[2] << std::endl;
//    file << "Maximum values: R = " << max_val[0] << ", G = " << max_val[1] << ", B = " << max_val[2] << std::endl;
//    file << "Minimum values: R = " << min_val[0] << ", G = " << min_val[1] << ", B = " << min_val[2] << std::endl;
//    file << std::endl;
//}
//
//// 将black_new_avg的数据保存到文件
//void save_to_file(const std::vector<std::vector<double>>& black_new_avg, const std::string& filename) {
//    std::ofstream file(filename);
//    if (!file.is_open()) {
//        std::cerr << "Failed to open file: " << filename << std::endl;
//        return;
//    }
//
//    // 保存统计信息
//    compute_statistics(black_new_avg, file);
//
//    // 保存black_new_avg的数据
//    file << "black_new_avg data:" << std::endl;
//    for (const auto& row : black_new_avg) {
//        for (double val : row) {
//            file << val << " ";
//        }
//        file << std::endl;
//    }
//
//    file.close();
//}
//
//// 主函数用于测试
//void black_show() {
//    // 假设black_avg是一个3507行3列的二维向量
//    std::vector<std::vector<double>> black_avg(3507, std::vector<double>(3, 0.0));
//    // 填充一些测试数据（这里只是示例，实际数据需要从文件读取）
//    for (int i = 0; i < 3507; ++i) {
//        black_avg[i][0] = i % 256; // R分量
//        black_avg[i][1] = (i + 100) % 256; // G分量
//        black_avg[i][2] = (i + 200) % 256; // B分量
//    }
//
//    // 调用函数处理
//    std::vector<std::vector<double>> black_new_avg = downsample_black_avg(black_avg);
//
//    // 将结果保存到文件
//    save_to_file(black_new_avg, "black_new_avg.txt");
//
//    //std::cout << "Results saved to black_new_avg.txt" << std::endl;
//
//    
//}