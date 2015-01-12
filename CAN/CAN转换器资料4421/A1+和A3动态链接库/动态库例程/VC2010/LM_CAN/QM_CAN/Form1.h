//以下是调用DLL声明
#include <windows.h>
HINSTANCE LM_DLL;
typedef int (__stdcall *P_Init_can)(unsigned char com_NUM,unsigned char Model,unsigned int CanBaudRate,unsigned char SET_ID_TYPE,unsigned char FILTER_MODE,unsigned char RXF[],unsigned char RXM[]);
typedef	int	(__stdcall *P_Can_send)(unsigned char IDbuff[],unsigned char Databuff[],unsigned char FreamType,unsigned char Bytes);
typedef	int	(__stdcall *P_Can_receive)(unsigned char IDbuff[],unsigned char Databuff[],unsigned char *FreamType,unsigned char *Bytes);
typedef	int	(__stdcall *P_Quit_can)(void);
P_Init_can Init_can;
P_Can_send Can_send;
P_Can_receive Can_receive;
P_Quit_can Quit_can;
//以上是调用DLL声明


#pragma once

namespace QM_CAN {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form1 摘要
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: 在此处添加构造函数代码
			//
		}

	protected:
		/// <summary>
		/// 清理所有正在使用的资源。
		/// </summary>
		~Form1()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  button1;
	protected: 
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Button^  button4;
	private: System::Windows::Forms::Button^  button5;
	private: System::Windows::Forms::TextBox^  textBox1;


	private:
		/// <summary>
		/// 必需的设计器变量。
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// 设计器支持所需的方法 - 不要
		/// 使用代码编辑器修改此方法的内容。
		/// </summary>
		void InitializeComponent(void)
		{
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(410, 45);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(79, 31);
			this->button1->TabIndex = 0;
			this->button1->Text = L"初始化DLL";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(410, 82);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(79, 31);
			this->button2->TabIndex = 1;
			this->button2->Text = L"连接设备";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(410, 119);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(79, 31);
			this->button3->TabIndex = 2;
			this->button3->Text = L"发送";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(410, 156);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(79, 31);
			this->button4->TabIndex = 3;
			this->button4->Text = L"接收";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &Form1::button4_Click);
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(410, 193);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(79, 31);
			this->button5->TabIndex = 4;
			this->button5->Text = L"释放设备";
			this->button5->UseVisualStyleBackColor = true;
			this->button5->Click += gcnew System::EventHandler(this, &Form1::button5_Click);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(12, 12);
			this->textBox1->Multiline = true;
			this->textBox1->Name = L"textBox1";
			this->textBox1->ScrollBars = System::Windows::Forms::ScrollBars::Vertical;
			this->textBox1->Size = System::Drawing::Size(369, 212);
			this->textBox1->TabIndex = 6;
			this->textBox1->TextChanged += gcnew System::EventHandler(this, &Form1::textBox1_TextChanged);
			// 
			// Form1
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(508, 251);
			this->Controls->Add(this->textBox1);
			this->Controls->Add(this->button5);
			this->Controls->Add(this->button4);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->button1);
			this->Name = L"Form1";
			this->Text = L"Form1";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
             //这里是动态加载 DLL函数例子。
	         LM_DLL = NULL;
	         LM_DLL = LoadLibrary("LM_USB.dll"); //动态方式加载dll
	         //error C2664: “LoadLibraryW”: 不能将参数 1 从“const char *”转换为“LPCWSTR”
	         //解决办法：选中当前工程->右键->属性->配置属性-->常规--->字符集---->使用多字节符字符集->确定

	         if (LM_DLL != NULL) {
		         Init_can = (P_Init_can)GetProcAddress(LM_DLL, "Init_can");
                 Can_send = (P_Can_send)GetProcAddress(LM_DLL, "Can_send");
                 Can_receive = (P_Can_receive)GetProcAddress(LM_DLL, "Can_receive");
                 Quit_can = (P_Quit_can)GetProcAddress(LM_DLL, "Quit_can");
		         textBox1 ->Text += L"加载DLL成功	\r\n";
	         }
	         else textBox1 ->Text += L"加载DLL失败\r\n";
			 //这里是动态加载 DLL函数例子。

		 	 textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
	
		 }
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
			 unsigned int ret;
             unsigned char RXF[4],RXM[4];
		     RXF[0] = 0; 
		     RXF[1] = 0;
		     RXF[2] = 0;
		     RXF[3] = 0;

             RXM[0] = 0;
             RXM[1] = 0;
		     RXM[2] = 0;
		     RXM[3] = 0;
		     ret = Init_can(0,1,20,0,0,RXF,RXM); //调用连接A1+设备函数例子
			 //ret 返回值 0：正常  1：已连接设备  2：无应答（端口有效无应答）3：无可用的串口（3-30）
		     if (ret==0) textBox1 ->Text += L"连接设备成功\r\n";
		     else textBox1 ->Text += L"连接设备失败，返回值"+ret+"\r\n";
		 	 textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }
private: System::Void button5_Click(System::Object^  sender, System::EventArgs^  e) {
             unsigned int ret;
		     ret = Quit_can(); //调用释放函数例子
		     textBox1 ->Text += L"释放设备连接\r\n";
		 	 textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) {
			 unsigned int ret;
             unsigned char IDbuff[4],Databuff[8],FreamType,Bytes;
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
		     ret = Can_send(IDbuff,Databuff,FreamType,Bytes); //调用发送函数例子
		     textBox1 ->Text += L"发送:"+IDbuff[0]+"  "+IDbuff[1]+"    "+Databuff[0]+"  "+Databuff[1]+"  "+Databuff[2]+"  "+Databuff[3]+"  "+Databuff[4]+"  "+Databuff[5]+"  "+Databuff[6]+"  "+Databuff[7]+"\r\n";
		     textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }
private: System::Void button4_Click(System::Object^  sender, System::EventArgs^  e) {
			 unsigned int ret;
			 unsigned char no_data;
             unsigned char IDbuff[4],Databuff[8],FreamType,Bytes;
			 no_data = 1;
		     do 
			 {
		         ret = Can_receive(IDbuff,Databuff,&FreamType,&Bytes);//调用接收函数例子
				 if (ret==1) 
				 {
					 textBox1 ->Text += L"接收:"+IDbuff[0]+"  "+IDbuff[1]+"    "+Databuff[0]+"  "+Databuff[1]+"  "+Databuff[2]+"  "+Databuff[3]+"  "+Databuff[4]+"  "+Databuff[5]+"  "+Databuff[6]+"  "+Databuff[7]+"\r\n";
					 no_data = 0;
				 }
			 }
             while (ret==1);
			 if (no_data ==1) textBox1 ->Text += L"接收: 无数据"+"\r\n";

		     textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }


private: System::Void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
};
}

