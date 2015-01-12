VERSION 5.00
Begin VB.Form Form1 
   Caption         =   "Form1"
   ClientHeight    =   6720
   ClientLeft      =   60
   ClientTop       =   450
   ClientWidth     =   8280
   LinkTopic       =   "Form1"
   ScaleHeight     =   6720
   ScaleWidth      =   8280
   StartUpPosition =   3  '窗口缺省
   Begin VB.TextBox Text1 
      Height          =   5775
      Left            =   240
      MultiLine       =   -1  'True
      ScrollBars      =   2  'Vertical
      TabIndex        =   4
      Text            =   "Form1.frx":0000
      Top             =   360
      Width           =   5775
   End
   Begin VB.CommandButton Command4 
      Caption         =   "释放设备"
      Height          =   375
      Left            =   6360
      TabIndex        =   3
      Top             =   3480
      Width           =   1335
   End
   Begin VB.CommandButton Command3 
      Caption         =   "接收"
      Height          =   375
      Left            =   6360
      TabIndex        =   2
      Top             =   2760
      Width           =   1335
   End
   Begin VB.CommandButton Command2 
      Caption         =   "发送"
      Height          =   495
      Left            =   6360
      TabIndex        =   1
      Top             =   1920
      Width           =   1335
   End
   Begin VB.CommandButton Command1 
      Caption         =   "连接设备"
      Height          =   495
      Left            =   6360
      TabIndex        =   0
      Top             =   1080
      Width           =   1335
   End
End
Attribute VB_Name = "Form1"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Private Declare Function Init_can Lib "LM_USB.dll" (ByVal com_NUM As Byte, ByVal Model As Byte, ByVal CanBaudRate As Integer, ByVal SET_ID_TYPE As Byte, ByVal FILTER_MODE As Byte, ByVal Ptr_RXF As Long, ByVal Ptr_RXM As Long) As Integer
Private Declare Function Can_send Lib "LM_USB.dll" (ByVal Ptr_IDbuff As Long, ByVal Ptr_Databuff As Long, ByVal FreamType As Byte, ByVal Bytes As Byte) As Integer
Private Declare Function Can_receive Lib "LM_USB.dll" (ByVal Ptr_IDbuff As Long, ByVal Ptr_Databuff As Long, ByRef FreamType As Byte, ByRef Bytes As Byte) As Integer
Private Declare Function Quit_can Lib "LM_USB.dll" () As Integer


Private Sub Command1_Click()
        Dim ret As Integer
        Dim RXF(0 To 4) As Byte
        Dim RXM(0 To 4) As Byte
        ret = Init_can(0, 3, 20, 0, 0, VarPtr(RXF(0)), VarPtr(RXM(0))) ' //调用连接A1+设备函数例子  VarPtr(RXF(0),VarPtr(RXM(0))
        'ret 返回值 0：正常  1：已连接设备  2：无应答（端口有效无应答）3：无可用的串口（3-30）
        If (ret = 0) Then
            Text1.Text = Text1.Text + "连接设备成功" & vbCrLf ' + Environment.NewLine
        Else
            Text1.Text = Text1.Text + "连接设备失败，返回值" + Str(ret) & vbCrLf ' + Environment.NewLine
        End If

       ' TextBox1.SelectionLength = 0
       ' TextBox1.SelectionStart = TextBox1.Text.Length
       'TextBox1.ScrollToCaret()

End Sub

Private Sub Command2_Click()
        Dim ret As Integer
        Dim IDbuff(0 To 3) As Byte
        Dim Databuff(0 To 7) As Byte
        Dim FreamType As Byte
        Dim Bytes As Byte
        IDbuff(0) = 0
        IDbuff(1) = 0
        Databuff(0) = 1
        Databuff(1) = 2
        Databuff(2) = 3
        Databuff(3) = 4
        Databuff(4) = 5
        Databuff(5) = 6
        Databuff(6) = 7
        Databuff(7) = 8

        FreamType = 0 '//标准数据帧
        Bytes = 8  ' //本帧8个字节
        ret = Can_send(VarPtr(IDbuff(0)), VarPtr(Databuff(0)), FreamType, Bytes) ' //调用发送函数例子
        Text1.Text = Text1.Text + "发送:" + Str(IDbuff(0)) + "  " + Str(IDbuff(1)) + "    " + Str(Databuff(0)) + "  " + Str(Databuff(1)) + "  " + Str(Databuff(2)) + "  " + Str(Databuff(3)) + "  " + Str(Databuff(4)) + "  " + Str(Databuff(5)) + "  " + Str(Databuff(6)) + "  " + Str(Databuff(7)) & vbCrLf  '+ Environment.NewLine
      '  TextBox1.SelectionLength = 0
      '  TextBox1.SelectionStart = TextBox1.Text.Length
      '  TextBox1.ScrollToCaret()

End Sub

Private Sub Command3_Click()
        Dim ret As Integer
        Dim no_data As Integer
        Dim IDbuff(4) As Byte
        Dim Databuff(8) As Byte
        Dim FreamType As Byte
        Dim Bytes As Byte
        ret = 1
        no_data = 1
        Do While (ret = 1)
            'ret = Can_receive(IDbuff, Databuff, FreamType, Bytes) '//调用接收函数例子
            ret = Can_receive(VarPtr(IDbuff(0)), VarPtr(Databuff(0)), FreamType, Bytes)

            If (ret = 1) Then
                Text1.Text = Text1.Text + "接收:" + Str(IDbuff(0)) + "  " + Str(IDbuff(1)) + "    " + Str(Databuff(0)) + "  " + Str(Databuff(1)) + "  " + Str(Databuff(2)) + "  " + Str(Databuff(3)) + "  " + Str(Databuff(4)) + "  " + Str(Databuff(5)) + "  " + Str(Databuff(6)) + "  " + Str(Databuff(7)) & vbCrLf '+ Environment.NewLine
                no_data = 0
            End If
        Loop
        If (no_data = 1) Then
            Text1.Text = Text1.Text + "接收: 无数据" & vbCrLf ' + Environment.NewLine
        End If

        'TextBox1.SelectionLength = 0
        'TextBox1.SelectionStart = TextBox1.Text.Length
        'TextBox1.ScrollToCaret()

End Sub

Private Sub Command4_Click()
    Dim ret As Integer
        ret = Quit_can() ' //调用释放函数例子
        Text1.Text = Text1.Text + "释放设备连接" & vbCrLf ' + Environment.NewLine
        'Text1.SelectionLength = 0
        'Text1.SelectionStart = TextBox1.Text.Length
        'Text1.ScrollToCaret()

End Sub

