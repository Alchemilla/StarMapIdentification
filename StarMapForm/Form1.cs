using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace StarMapForm
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        /// <summary>
        /// 界面加载时，主动加载的部分
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Form1_Load(object sender, EventArgs e)
        {
            textBox2.Text = @"D:\2_ImageData\ZY3-02\星图处理\0707\星图\控制点\Allgcp.txt";
            textBox3.Text = "D:\\2_ImageData\\ZY3-02\\星图处理\\0707\\0707.STG";
            textBox4.Text = @"D:\2_ImageData\ZY3-02\1-定位精度\579";
        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                textBox2.Text = openDlg.FileName.ToString(); //获得文件路径                                
                ShowInfo("成功打开星图控制点");
            }
            else { ShowInfo("未选择星点数据路径"); }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                textBox3.Text = openDlg.FileName.ToString(); //获得文件路径                
                ShowInfo("成功打开STG文件");
            }
            else { ShowInfo("未打开STG文件"); }
        }

        private void button3_Click(object sender, EventArgs e)
        {
            OpenFileDialog openDlg = new OpenFileDialog();
            if (openDlg.ShowDialog() == DialogResult.OK)
            {
                textBox4.Text = openDlg.FileName.ToString(); //获得文件路径                
                ShowInfo("成功设置输出文件路径");
            }
            else { ShowInfo("未设置输出文件路径"); }
        }
        /// <summary>
        /// 星图姿态确定
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button4_Click(object sender, EventArgs e)
        {
            Process exep = new Process();
            exep.StartInfo.FileName = AppDomain.CurrentDomain.BaseDirectory + "ZY3-StarSensor.exe";
            exep.StartInfo.Arguments = "1 " + textBox2.Text + " " + textBox3.Text;
            exep.StartInfo.CreateNoWindow = true;
            exep.StartInfo.UseShellExecute = false;
            ShowInfo("根据星点矢量进行姿态确定\n开始处理...");
            exep.Start();
            exep.WaitForExit();
            int returnValue = exep.ExitCode;
            if (returnValue == 0)
            { ShowInfo("星图数据处理完毕"); }
            else
            { ShowInfo("星图数据有问题"); }
        }

        //信息实时显示
        public void ShowInfo(string Info)
        {
            textBox1.AppendText(DateTime.Now.ToString("HH:mm:ss  ") + Info + "\r\n");
            textBox1.SelectionStart = textBox1.Text.Length; //设定光标位置
            textBox1.ScrollToCaret(); //滚动到光标处
        }
        /// <summary>
        /// 严密模型
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button5_Click(object sender, EventArgs e)
        {
            string sourcePath = textBox3.Text.Substring(0,textBox3.Text.LastIndexOf('\\')+1)+"StarMap滤波结果.txt";
            string targetPath = textBox4.Text + "\\"+
                textBox4.Text.Substring(textBox4.Text.LastIndexOf('\\') + 1)+".ATT";
            System.IO.File.Copy(sourcePath, targetPath, true);
            Process exep = new Process();
            exep.StartInfo.FileName = AppDomain.CurrentDomain.BaseDirectory + "GeometryProcess.exe";
            exep.StartInfo.Arguments = textBox4.Text +" 1";
            exep.StartInfo.CreateNoWindow = true;
            exep.StartInfo.UseShellExecute = false;
            ShowInfo("几何模型开始处理...");
            exep.Start();
            exep.WaitForExit();
            int returnValue = exep.ExitCode;
            if (returnValue == 0)
            { ShowInfo("几何模型处理完毕"); }
            else
            { ShowInfo("数据有问题"); }
        }
        /// <summary>
        /// 模型外定标
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button6_Click(object sender, EventArgs e)
        {
            string sourcePath = textBox3.Text.Substring(0, textBox3.Text.LastIndexOf('\\') + 1) + "StarMap滤波结果.txt";
            string targetPath = textBox4.Text + "\\" +
                textBox4.Text.Substring(textBox4.Text.LastIndexOf('\\') + 1) + ".ATT";
            System.IO.File.Copy(sourcePath, targetPath, true);
            Process exep = new Process();
            exep.StartInfo.FileName = AppDomain.CurrentDomain.BaseDirectory + "GeometryProcess.exe";
            exep.StartInfo.Arguments = textBox4.Text + " 2";
            exep.StartInfo.CreateNoWindow = true;
            exep.StartInfo.UseShellExecute = false;
            ShowInfo("几何定标开始处理...");
            exep.Start();
            exep.WaitForExit();
            int returnValue = exep.ExitCode;
            if (returnValue == 0)
            { ShowInfo("几何模型处理完毕"); }
            else
            { ShowInfo("数据有问题"); }
        }

        /// <summary>
        /// 对比各种结果
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void button7_Click(object sender, EventArgs e)
        {
            string workpath = textBox3.Text.Substring(0, textBox3.Text.LastIndexOf('\\') + 1);
            Process exep = new Process();
            exep.StartInfo.FileName = AppDomain.CurrentDomain.BaseDirectory + "ZY3-StarSensor.exe";
            exep.StartInfo.Arguments = workpath + " 11";
            exep.StartInfo.CreateNoWindow = true;
            exep.StartInfo.UseShellExecute = false;
            ShowInfo("几何定标开始处理...");
            exep.Start();
            exep.WaitForExit();
            int returnValue = exep.ExitCode;
            if (returnValue == 0)
            { ShowInfo("几何模型处理完毕"); }
            else
            { ShowInfo("数据有问题"); }
        }
    }


}
