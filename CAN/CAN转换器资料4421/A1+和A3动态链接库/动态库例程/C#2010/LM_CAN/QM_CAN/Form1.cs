using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;          



namespace QM_CAN
{
    public partial class Form1 : Form
    {
        //以下是加载 DLL声明部分
        IntPtr QM_DLL; 
        TYPE_Init_can Init_can;
        TYPE_Quit_can Quit_can;
        TYPE_Can_send Can_send;
        TYPE_Can_receive Can_receive;
        delegate int  TYPE_Init_can(byte com_NUM, byte Model, int CanBaudRate, byte SET_ID_TYPE, byte FILTER_MODE, byte[] RXF, byte[] RXM);
        delegate int TYPE_Quit_can();
        delegate int TYPE_Can_send(byte[] IDbuff, byte[] Databuff, byte FreamType, byte Bytes);
        delegate int TYPE_Can_receive(byte[] IDbuff, byte[] Databuff, byte[] FreamType, byte[] Bytes);        
        [DllImport("kernel32.dll")] static extern IntPtr LoadLibrary(string lpFileName); 
        [DllImport("kernel32.dll")] static extern IntPtr GetProcAddress(IntPtr hModule, string lpProcName);
        //以上是加载 DLL声明部分

        public Form1()
        {            
            InitializeComponent();
        }    
        
        private void button1_Click(object sender, EventArgs e)
        {
            //这里是动态加载 DLL函数例子。
            QM_DLL = LoadLibrary("LM_USB.dll");
            if (QM_DLL != IntPtr.Zero)
            {
                IntPtr P_Init_can = GetProcAddress(QM_DLL, "Init_can");
                IntPtr P_Quit_can = GetProcAddress(QM_DLL, "Quit_can");
                IntPtr P_Can_send = GetProcAddress(QM_DLL, "Can_send");
                IntPtr P_Can_receive = GetProcAddress(QM_DLL, "Can_receive");
                Init_can = (TYPE_Init_can)Marshal.GetDelegateForFunctionPointer(P_Init_can, typeof(TYPE_Init_can));
                Quit_can = (TYPE_Quit_can)Marshal.GetDelegateForFunctionPointer(P_Quit_can, typeof(TYPE_Quit_can));
                Can_send = (TYPE_Can_send)Marshal.GetDelegateForFunctionPointer(P_Can_send, typeof(TYPE_Can_send));
                Can_receive = (TYPE_Can_receive)Marshal.GetDelegateForFunctionPointer(P_Can_receive, typeof(TYPE_Can_receive));
		         textBox1.Text += "加载DLL成功	\r\n";
            }
            else textBox1.Text += "加载DLL失败\r\n";
            //这里是动态加载 DLL函数例子。
            textBox1.SelectionLength = 0;
            textBox1.SelectionStart = textBox1.Text.Length;
            textBox1.ScrollToCaret();
        }

        private void button2_Click(object sender, EventArgs e)
        {
             int ret;
             byte[] RXF = new byte[4] { 1, 2, 3, 4 };
             byte[] RXM = new byte[4] { 1, 2, 3, 4 };
		     RXF[0] = 0; 
		     RXF[1] = 0;
		     RXF[2] = 0;
		     RXF[3] = 0;

             RXM[0] = 0;
             RXM[1] = 0;
		     RXM[2] = 0;
		     RXM[3] = 0;
             ret = Init_can(0, 1, 20, 0, 0, RXF, RXM); //调用连接A1+设备函数例子
			 //ret 返回值 0：正常  1：已连接设备  2：无应答（端口有效无应答）3：无可用的串口（3-30）
		     if (ret==0) textBox1.Text += "连接设备成功\r\n";
		     else textBox1.Text += "连接设备失败，返回值"+ret+"\r\n";

		 	 textBox1.SelectionLength = 0;
             textBox1.SelectionStart = textBox1.Text.Length;
             textBox1.ScrollToCaret() ;
        }

        private void button3_Click(object sender, EventArgs e)
        {
            
          	int ret;
            byte[] IDbuff = new byte[4] { 0, 0, 0, 0 };
            byte[] Databuff = new byte[8] { 0, 0, 0, 0, 0, 0, 0, 0};
            byte FreamType,Bytes;
		    IDbuff[0] = 0;
		    IDbuff[1] = 0;
		    Databuff[0] = 1;
		    Databuff[1] = 2;
		    Databuff[2] = 3;
		    Databuff[3] = 4;
		    Databuff[4] = 5;
		    Databuff[5] = 6;
		    Databuff[6] = 7;
		    Databuff[7] = 8;
		    FreamType = 0;//标准数据帧
		    Bytes = 8; //本帧8个字节
		    ret=Can_send(IDbuff,Databuff,FreamType,Bytes); //调用发送函数例子
		    textBox1.Text += "发送:"+IDbuff[0]+"  "+IDbuff[1]+"    "+Databuff[0]+"  "+Databuff[1]+"  "+Databuff[2]+"  "+Databuff[3]+"  "+Databuff[4]+"  "+Databuff[5]+"  "+Databuff[6]+"  "+Databuff[7]+"\r\n";
             

            textBox1.SelectionLength = 0;
            textBox1.SelectionStart = textBox1.Text.Length;
            textBox1.ScrollToCaret();            
        }

        private void button4_Click(object sender, EventArgs e)
        {
            
            int ret;
            byte no_data;
            byte[] IDbuff = new byte[4];
            byte[] Databuff = new byte[8];
            byte[] FreamType = new byte[1];
            byte[] Bytes = new byte[1];
            no_data = 1;
		    do 
			{
		        ret = Can_receive(IDbuff,Databuff,FreamType,Bytes);//调用接收函数例子
                if (ret == 1)
                {
                    textBox1.Text += "接收:" + IDbuff[0] + "  " + IDbuff[1] + "    " + Databuff[0] + "  " + Databuff[1] + "  " + Databuff[2] + "  " + Databuff[3] + "  " + Databuff[4] + "  " + Databuff[5] + "  " + Databuff[6] + "  " + Databuff[7] + "\r\n";
                    no_data = 0;
                }

			}
            while (ret==1);
            if (no_data == 1) textBox1.Text += "接收: 无数据" + "\r\n";
            textBox1.SelectionLength = 0;
            textBox1.SelectionStart = textBox1.Text.Length;
            textBox1.ScrollToCaret();
        }

        private void button5_Click(object sender, EventArgs e)
        {
            int ret;
		    ret = Quit_can(); //调用释放函数例子
		    textBox1.Text += "释放设备连接\r\n";
            
            textBox1.SelectionLength = 0;
            textBox1.SelectionStart = textBox1.Text.Length;
            textBox1.ScrollToCaret();
        }
    }
}
