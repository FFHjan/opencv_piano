//Opencv3.4.10
//
//���낢�뒲�����鎞��22�s�ڂ�270�s�ڂ̃R�����g��ǂ�ł�
//git
#include "thread"
#include "math.h"
#include "cstdio"
#include "cstdint"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
using namespace cv;

void camera_setting(void);
void L(void);
void R(void);
void HSVCal(void);
void shokikaL(void);
void shokikaR(void);

const int LCamera_ID = 1, RCamera_ID = 0;
static cv::VideoCapture capL(CAP_MSMF + LCamera_ID);//fps�̏o��MSMF�͋N���ɂ߂��Ⴍ���᎞�Ԃ�������̂�(1���ォ����Ƃ�������)�f�o�b�O�̎��͉�2�s��DSHOW�ł�����ق����悳��
static cv::VideoCapture capR(CAP_MSMF + RCamera_ID);
//static VideoCapture capL(CAP_DSHOW+LCamera_ID);
//static VideoCapture capR(CAP_DSHOW +RCamera_ID);
static Mat frameL, frameR; //�擾�����t���[��

const int RHmin = 320, RHmax = 3, RSmin = 70, RVmin = 50, OHmin = 10, OHmax = 25, OSmin = 60, OVmin = 60, O2Hmin = 29, O2Hmax = 40, O2Smin = 50, O2Vmin = 60, YHmin = 50, YHmax = 65, YSmin = 50, YVmin = 60, GHmin = 85, GHmax = 150, GSmin = 30, GVmin = 55, BHmin = 200, BHmax = 220, BSmin = 65, BVmin = 50;
//public const int RHmin = 340, RHmax = 7, RSmin = 500, RVmin = 500, OHmin = 10, OHmax = 25, OSmin = 600, OVmin = 700, O2Hmin = 29, O2Hmax = 40, O2Smin = 600, O2Vmin = 600, YHmin = 50, YHmax = 65, YSmin = 600, YVmin = 700, GHmin = 85, GHmax = 120, GSmin = 400, GVmin = 500, BHmin = 200, BHmax = 220, BSmin = 60, BVmin = 60;
const int dotsizemax = 18, dotsizemin = 6, dotsumin = 20, toomuchnoise = 36, skipdot = 5, width = 1280, height = 720, FPS = 60;
const double distansemin = 36;
static short hsv[256][256][256][3];
static float LCamera[6][toomuchnoise][4], RCamera[6][toomuchnoise][4];//�J���[�ԍ�0�ԁA�}�[�J�[�̐��Axyz+�ԈȊO�������l�ɂȂ��Ă��邩�`�F�b�N����v�f
static float test[10][10];
static int aroundx[8] = { -1, 0, 1, 1, 1, 0, -1, -1};
static int aroundy[8] = { -1, -1, -1, 0, 1, 1, 1, 0};
static int Markerhsv[6][4] = { {RHmin,RHmax,RSmin,RVmin},{OHmin,OHmax,OSmin,OVmin},{O2Hmin,O2Hmax,O2Smin,O2Vmin},{YHmin,YHmax,YSmin,YVmin},{GHmin,GHmax,GSmin,GVmin},{BHmin,BHmax,BSmin,BVmin} };

void search(int LorR)
{
    Mat frame;
    if (LorR == 0) frame = frameL;
    else frame = frameR;

    Vec3b* src = frame.ptr<Vec3b>(0);
    for (int i = 0; i < frame.cols; i++)
    {
        src[i] = Vec3b(0, 0, 0);
    }
    src = frame.ptr<Vec3b>(height / 2 - 1);
    for (int i = 0; i < frame.cols; i++)
    {
        src[i] = Vec3b(0, 0, 0);
    }
    for (int i = 1; i < frame.rows / 2 - 1; i++) frame.at<Vec3b>(i, 0) = Vec3b(0, 0, 0);
    for (int i = 1; i < frame.rows / 2 - 1; i++) frame.at<Vec3b>(i, frame.cols - 1) = Vec3b(0, 0, 0);
    int num = 1, xmax = 0, xmin = 0, ymax = 0, ymin = 0, xsum = 0, ysum = 0, dotsum = 0;
    int loopcount[6] = { 0,0,0,0,0,0 };
    bool noiseflag = false;
    for (int i = skipdot; i < frame.rows / 2 - 1; i += skipdot)
    {
        if (noiseflag == true) {
            printf("�}�[�J�[�Ɠ����F�̕��̂��������܂�\n");
            break;
        }
        src = frame.ptr<Vec3b>(i);
        for (int j = skipdot; j < frame.cols; j +=skipdot)
        {
            int H = (int)hsv[src[j][2]][src[j][1]][src[j][0]][0];
            int S = (int)hsv[src[j][2]][src[j][1]][src[j][0]][1];
            int V = (int)hsv[src[j][2]][src[j][1]][src[j][0]][2];
            if ((H >= RHmin || H <= RHmax) && S >= RSmin && V >= RVmin)//�Ԍ��o////////////////////////////////////////////////////////////////////
            {
                int p = j, q = i, k, tmpx, tmpy;
                bool iso = true;
                while (((int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][0] >= RHmin || (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][0] <= RHmax) && (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][1] >= RSmin && (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][2] >= RVmin || ((int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][0] >= RHmin || (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][0] <= RHmax) && (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][1] >= RSmin && (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][2] >= RVmin)
                {
                    p--;
                }//��΂���΂��`�F�b�N���Ă�̂ŏ����ɍ����_������������_�Q�̒[��������܂Ō�ނ���B(���΂��Γ_�Q�̒���1�h�b�g���������O�̃h�b�g������̂ł�����z���邽��2�O�̃h�b�g���`�F�b�N����)
                    //���łɂ���_�Q�Ƌ����`�F�b�N
                bool checkdis = true;
                if (LorR == 0)
                {
                    for (int l = 0; LCamera[0][l][0] != 0; l++)
                    {
                        if (abs(LCamera[0][l][0] - (float)p) <= distansemin) checkdis = false;
                    }
                }
                else
                {
                    for (int l = 0; RCamera[0][l][0] != 0; l++)
                    {
                        if (abs(RCamera[0][l][0] - (float)p) <= distansemin) checkdis = false;
                    }
                }
                if (checkdis == true)
                {
                    tmpx = p;
                    tmpy = q;
                    for (k = 0; k < 8; k++)  //�ŏ���1�h�b�gor�Ǘ��_����
                    {
                        if ((hsv[frame.at<Vec3b>(q+aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] >= RHmin || hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] <= RHmax) && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][1] >= RSmin && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][2] >= RVmin)
                        {
                            frame.at<Vec3b>(q, p) = Vec3b(255, 255, num);
                            xmin = xmax = xsum = p;
                            ymin = ymax = ysum = q;
                            dotsum = 1;
                            p += aroundx[k];
                            q += aroundy[k];
                            iso = false;
                            break;
                        }
                    }
                    if (iso == true)//�Ǘ��_������skipdot�ȏ�߂��čđ�(����Ȃ��Ƃ��Ȃ��Ă�j=p�ł����C�����Ă���)
                    {
                        frame.at<Vec3b>(q, p) = Vec3b(0, 0, 0);
                        j -= (j - p) / skipdot + skipdot;
                    }
                    else
                    {
                        do
                        {
                            k = (k + 5) % 8;
                            while (true)
                            {
                                if ((hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] >= RHmin || hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] <= RHmax) && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][1] >= RSmin && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][2] >= RVmin || frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2] == num && frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1] == 255 && frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0] == 255)
                                {
                                    frame.at<Vec3b>(q, p) = Vec3b(255, 255, num);
                                    if (xmin > p ) xmin = p;
                                    if (xmax < p ) xmax = p;
                                    if (ymin > q ) ymin = q;
                                    if (ymax < q ) ymax = q;
                                    xsum += p;
                                    ysum += q;
                                    dotsum++;
                                    p += aroundx[k];
                                    q += aroundy[k];
                                    break;
                                }
                                k = (k + 1) % 8;
                            }
                        } while (tmpx != p|| tmpy != q);
                        if (xmax - xmin >= dotsizemin && xmax - xmin <= dotsizemax && ymax - ymin >= dotsizemin && ymax - ymin <= dotsizemax && dotsum > dotsumin)
                        {
                            if (LorR == 0) {
                                LCamera[0][loopcount[0]][0] = (float)xsum / (float)dotsum;
                                LCamera[0][loopcount[0]][1] = (float)ysum / (float)dotsum;
                                loopcount[0]++;
                            }
                            else
                            {
                                RCamera[0][loopcount[0]][0] = (float)xsum / (float)dotsum;
                                RCamera[0][loopcount[0]][1] = (float)ysum / (float)dotsum;
                                loopcount[0]++;
                            }
                        }
                        num++;
                    }
                }
            }
            //���̐F(�ԂƑ��̐F�ŏ��������Ⴄ�̂ł������Ȃ������悤�ȃR�[�h���Q����)///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            for (int c = 1; c < 6; c++) {
                if (H >= Markerhsv[c][0] && H <= Markerhsv[c][1] && S >= Markerhsv[c][2] && V >= Markerhsv[c][3])//�Ԍ��o////////////////////////////////////////////////////////////////////
                {
                    int p = j, q = i, k, tmpx, tmpy;
                    bool iso = true;
                    while (((int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][0] >= Markerhsv[c][0] || (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][0] <= Markerhsv[c][2]) && (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][1] >= Markerhsv[c][2] && (int)hsv[frame.at<Vec3b>(q, p - 1)[2]][frame.at<Vec3b>(q, p - 1)[1]][frame.at<Vec3b>(q, p - 1)[0]][2] >= Markerhsv[c][3] || ((int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][0] >= Markerhsv[c][0] || (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][0] <= Markerhsv[c][1]) && (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][1] >= Markerhsv[c][2] && (int)hsv[frame.at<Vec3b>(q, p - 2)[2]][frame.at<Vec3b>(q, p - 2)[1]][frame.at<Vec3b>(q, p - 2)[0]][2] >= Markerhsv[c][3])
                    {
                        p--;
                    }//��΂���΂��`�F�b�N���Ă�̂ŏ����ɍ����_������������_�Q�̒[��������܂Ō�ނ���B(���΂��Γ_�Q�̒���1�h�b�g���������O�̃h�b�g������̂ł�����z���邽��2�O�̃h�b�g���`�F�b�N����)
                    //���łɂ���_�Q�Ƌ����`�F�b�N
                    bool checkdis = true;
                    if (LorR == 0)
                    {
                        for (int l = 0; LCamera[c][l][0] != 0; l++)
                        {
                            if (abs(LCamera[c][l][0] - (float)p) <= distansemin) checkdis = false;
                        }
                    }
                    else
                    {
                        for (int l = 0; RCamera[c][l][0] != 0; l++)
                        {
                            if (abs(RCamera[c][l][0] - (float)p) <= distansemin) checkdis = false;
                        }
                    }
                    if (checkdis == true)
                    {
                        tmpx = p;
                        tmpy = q;
                        for (k = 0; k < 8; k++)  //�ŏ���1�h�b�gor�Ǘ��_����
                        {
                            if ((hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] >= Markerhsv[c][0] || hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] <= Markerhsv[c][1]) && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][1] >= Markerhsv[c][2] && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][2] >= Markerhsv[c][3])
                            {
                                frame.at<Vec3b>(q, p) = Vec3b(255, 255, num);
                                xmin = xmax = xsum = p;
                                ymin = ymax = ysum = q;
                                dotsum = 1;
                                p += aroundx[k];
                                q += aroundy[k];
                                iso = false;
                                break;
                            }
                        }
                        if (iso == true)//�Ǘ��_������skipdot�ȏ�߂��čđ�(����Ȃ��Ƃ��Ȃ��Ă�j=p�ł����C�����Ă���)
                        {
                            frame.at<Vec3b>(q, p) = Vec3b(0, 0, 0);
                            j -= (j - p) / skipdot + skipdot;
                        }
                        else
                        {
                            do
                            {
                                k = (k + 5) % 8;
                                while (true)
                                {
                                    if ((hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] >= Markerhsv[c][0] || hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][0] <= Markerhsv[c][1]) && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][1] >= Markerhsv[c][2] && hsv[frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1]][frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0]][2] >= Markerhsv[c][3] || frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[2] == num && frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[1] == 255 && frame.at<Vec3b>(q + aroundy[k], p + aroundx[k])[0] == 255)
                                    {
                                        frame.at<Vec3b>(q, p) = Vec3b(255, 255, num);
                                        if (xmin > p) xmin = p;
                                        if (xmax < p) xmax = p;
                                        if (ymin > q) ymin = q;
                                        if (ymax < q) ymax = q;
                                        xsum += p;
                                        ysum += q;
                                        dotsum++;
                                        p += aroundx[k];
                                        q += aroundy[k];
                                        break;
                                    }
                                    k = (k + 1) % 8;
                                }
                            } while (tmpx != p || tmpy != q);
                            if (xmax - xmin >= dotsizemin && xmax - xmin <= dotsizemax && ymax - ymin >= dotsizemin && ymax - ymin <= dotsizemax && dotsum > dotsumin)
                            {
                                if (LorR == 0) {
                                    LCamera[c][loopcount[c]][0] = (float)xsum / (float)dotsum;
                                    LCamera[c][loopcount[c]][1] = (float)ysum / (float)dotsum;
                                    loopcount[c]++;
                                }
                                else
                                {
                                    RCamera[c][loopcount[c]][0] = (float)xsum / (float)dotsum;
                                    RCamera[c][loopcount[c]][1] = (float)ysum / (float)dotsum;
                                    loopcount[c]++;
                                }
                            }
                            num++;
                        }
                    }
                }
            }
            if (num == toomuchnoise)
            {
                noiseflag = true;
                break;
            }
        }
    }

    if (LorR == 0)//imshow�͊m���ɒx����waitKey���x���炵���Ƃ����m���𓾂��̂Ŏ��p�Ɏg���Ƃ��͂���if��else���R�����g�A�E�g���Ă�
    {
        //LCamera[][][]���̃f�[�^��unity�ɑ��肽��
    }
    else
    {
        //RCamera[][][]���̃f�[�^��unity�ɑ��肽��
    }

    //if (LorR == 0)//imshow�͊m���ɒx����waitKey���x���炵���Ƃ����m���𓾂��̂Ńf�o�b�O�̎��͂���if��else�̃R�����g�A�E�g���������Ă�
    //{
    //    imshow("L", frame);
    //    waitKey(1);
    //}
    //else
    //{
    //    imshow("R", frame);
    //    waitKey(1);
    //}
    
}

void L()
{
    while (capL.read(frameL))//�������[�v
    {
        search(0);
        shokikaL();
    }
}

void R()
{
    while (capR.read(frameR))//�������[�v
    {
        search(1);
        shokikaR();
    }
}

int main(int argh, char* argv[])
{
    int count = 0;
    printf("start");
    HSVCal();
    printf("finish");

    //�f�o�C�X�̃I�[�v��

    if (!capL.isOpened() || !capR.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
    {
        //�ǂݍ��݂Ɏ��s�����Ƃ��̏���
        return -1;
    }
    camera_setting();

    std::thread L(L);
    std::thread R(R);

    L.join();
    R.join();
    
    
    destroyAllWindows();
    return 0;
}

void camera_setting(void)
{
    printf("%d\n", capL.set(CAP_PROP_FRAME_WIDTH, width));
    capL.set(CAP_PROP_FRAME_HEIGHT, height);
    capL.set(CAP_PROP_FPS, FPS);
    capL.set(CAP_PROP_BUFFERSIZE, 1);
    printf("%f\n",capL.get(CAP_PROP_FPS));

    printf("%d\n", capR.set(CAP_PROP_FRAME_WIDTH, width));
    capR.set(CAP_PROP_FRAME_HEIGHT, height);
    capR.set(CAP_PROP_FPS, FPS);
    capR.set(CAP_PROP_BUFFERSIZE, 1);
    printf("%f\n", capR.get(CAP_PROP_FPS));
}

void shokikaL(void) {
    for (int i = 0; i != 6; i++) {
        for (int j = 0; j != toomuchnoise; j++) {
            for (int k = 0; k != 4; k++) {
                LCamera[i][j][k] = 0;
            }
        }
    }
}

void shokikaR(void) {
    for (int i = 0; i != 6; i++) {
        for (int j = 0; j != toomuchnoise; j++) {
            for (int k = 0; k != 4; k++) {
                RCamera[i][j][k] = 0;
            }
        }
    }
}

void HSVCal()
{
    int r, g, b;
    double R, G, B, H = 0, S = 0, V = 0;
    for (r = 0; r != 256; r++)
    {
        for (g = 0; g != 256; g++)
        {
            for (b = 0; b != 256; b++)
            {
                R = r;
                G = g;
                B = b;
                double MAX = max(max(R, G), B);
                double MIN = min(min(R, G), B);
                V = MAX / 256 * 100;

                if (MAX == MIN)
                {
                    H = 0;
                    S = 0;
                }
                else
                {
                    if (MAX == R) H = 60.0 * (G - B) / (MAX - MIN) + 0;
                    else if (MAX == G) H = 60.0 * (B - R) / (MAX - MIN) + 120.0;
                    else if (MAX == B) H = 60.0 * (R - G) / (MAX - MIN) + 240.0;

                    if (H > 360.0) H = H - 360.0;
                    else if (H < 0) H = H + 360.0;
                    S = (MAX - MIN) / MAX * 100.0;
                }
                hsv[r][g][b][0] = (short)H;
                hsv[r][g][b][1] = (short)S;
                hsv[r][g][b][2] = (short)V;
            }
        }
    }
}
double max(double x, double y)
{
    if (x >= y)
    {
        return x;
    }
    else
    {
        return y;
    }
}
double min(double x, double y)
{
    if (x <= y)
    {
        return x;
    }
    else
    {
        return y;
    }
}