//�����ǵ���DLL����
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
//�����ǵ���DLL����


#pragma once

namespace QM_CAN {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Form1 ժҪ
	/// </summary>
	public ref class Form1 : public System::Windows::Forms::Form
	{
	public:
		Form1(void)
		{
			InitializeComponent();
			//
			//TODO: �ڴ˴���ӹ��캯������
			//
		}

	protected:
		/// <summary>
		/// ������������ʹ�õ���Դ��
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
		/// ����������������
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// �����֧������ķ��� - ��Ҫ
		/// ʹ�ô���༭���޸Ĵ˷��������ݡ�
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
			this->button1->Text = L"��ʼ��DLL";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &Form1::button1_Click);
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(410, 82);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(79, 31);
			this->button2->TabIndex = 1;
			this->button2->Text = L"�����豸";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &Form1::button2_Click);
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(410, 119);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(79, 31);
			this->button3->TabIndex = 2;
			this->button3->Text = L"����";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &Form1::button3_Click);
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(410, 156);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(79, 31);
			this->button4->TabIndex = 3;
			this->button4->Text = L"����";
			this->button4->UseVisualStyleBackColor = true;
			this->button4->Click += gcnew System::EventHandler(this, &Form1::button4_Click);
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(410, 193);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(79, 31);
			this->button5->TabIndex = 4;
			this->button5->Text = L"�ͷ��豸";
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
             //�����Ƕ�̬���� DLL�������ӡ�
	         LM_DLL = NULL;
	         LM_DLL = LoadLibrary("LM_USB.dll"); //��̬��ʽ����dll
	         //error C2664: ��LoadLibraryW��: ���ܽ����� 1 �ӡ�const char *��ת��Ϊ��LPCWSTR��
	         //����취��ѡ�е�ǰ����->�Ҽ�->����->��������-->����--->�ַ���---->ʹ�ö��ֽڷ��ַ���->ȷ��

	         if (LM_DLL != NULL) {
		         Init_can = (P_Init_can)GetProcAddress(LM_DLL, "Init_can");
                 Can_send = (P_Can_send)GetProcAddress(LM_DLL, "Can_send");
                 Can_receive = (P_Can_receive)GetProcAddress(LM_DLL, "Can_receive");
                 Quit_can = (P_Quit_can)GetProcAddress(LM_DLL, "Quit_can");
		         textBox1 ->Text += L"����DLL�ɹ�	\r\n";
	         }
	         else textBox1 ->Text += L"����DLLʧ��\r\n";
			 //�����Ƕ�̬���� DLL�������ӡ�

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
		     ret = Init_can(0,1,20,0,0,RXF,RXM); //��������A1+�豸��������
			 //ret ����ֵ 0������  1���������豸  2����Ӧ�𣨶˿���Ч��Ӧ��3���޿��õĴ��ڣ�3-30��
		     if (ret==0) textBox1 ->Text += L"�����豸�ɹ�\r\n";
		     else textBox1 ->Text += L"�����豸ʧ�ܣ�����ֵ"+ret+"\r\n";
		 	 textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }
private: System::Void button5_Click(System::Object^  sender, System::EventArgs^  e) {
             unsigned int ret;
		     ret = Quit_can(); //�����ͷź�������
		     textBox1 ->Text += L"�ͷ��豸����\r\n";
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
		     FreamType = 0;//��׼����֡
		     Bytes = 8; //��֡8���ֽ�
		     ret = Can_send(IDbuff,Databuff,FreamType,Bytes); //���÷��ͺ�������
		     textBox1 ->Text += L"����:"+IDbuff[0]+"  "+IDbuff[1]+"    "+Databuff[0]+"  "+Databuff[1]+"  "+Databuff[2]+"  "+Databuff[3]+"  "+Databuff[4]+"  "+Databuff[5]+"  "+Databuff[6]+"  "+Databuff[7]+"\r\n";
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
		         ret = Can_receive(IDbuff,Databuff,&FreamType,&Bytes);//���ý��պ�������
				 if (ret==1) 
				 {
					 textBox1 ->Text += L"����:"+IDbuff[0]+"  "+IDbuff[1]+"    "+Databuff[0]+"  "+Databuff[1]+"  "+Databuff[2]+"  "+Databuff[3]+"  "+Databuff[4]+"  "+Databuff[5]+"  "+Databuff[6]+"  "+Databuff[7]+"\r\n";
					 no_data = 0;
				 }
			 }
             while (ret==1);
			 if (no_data ==1) textBox1 ->Text += L"����: ������"+"\r\n";

		     textBox1->SelectionLength = 0;
             textBox1->SelectionStart = textBox1->Text->Length;
             textBox1->ScrollToCaret() ;
		 }


private: System::Void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e) {
		 }
};
}

