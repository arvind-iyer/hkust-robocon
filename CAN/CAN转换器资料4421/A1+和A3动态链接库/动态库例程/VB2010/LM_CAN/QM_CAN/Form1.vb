
Imports System.Runtime.InteropServices

Public Class Form1
    Dim LM_DLL As IntPtr
    Dim P_Init_can As IntPtr
    Dim P_Quit_can As IntPtr
    Dim P_Can_send As IntPtr
    Dim P_Can_receive As IntPtr

    Delegate Function TYPE_Init_can(ByVal com_NUM As Byte, ByVal Model As Byte, ByVal CanBaudRate As Integer, ByVal SET_ID_TYPE As Byte, ByVal FILTER_MODE As Byte, ByVal RXF() As Byte, ByVal RXM() As Byte) As Integer
    Delegate Function TYPE_Quit_can() As Integer
    Delegate Function TYPE_Can_send(ByVal IDbuff() As Byte, ByVal Databuff() As Byte, ByVal FreamType As Byte, ByVal Bytes As Byte) As Integer
    Delegate Function TYPE_Can_receive(ByVal IDbuff() As Byte, ByVal Databuff() As Byte, ByVal FreamType() As Byte, ByVal Bytes() As Byte) As Integer

    Dim Init_can As TYPE_Init_can
    Dim Quit_can As TYPE_Quit_can
    Dim Can_send As TYPE_Can_send
    Dim Can_receive As TYPE_Can_receive

    Public Declare Function LoadLibrary Lib "kernel32" Alias "LoadLibraryA" (ByVal lpLibFileName As String) As IntPtr
    Public Declare Function GetProcAddress Lib "kernel32" (ByVal hModule As IntPtr, ByVal lpProcName As String) As IntPtr

    Private Sub Button1_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button1.Click

        LM_DLL = LoadLibrary("LM_USB.dll") '动态方式加载dll
        'error C2664: “LoadLibraryW”: 不能将参数 1 从“const char *”转换为“LPCWSTR”
        '解决办法：选中当前工程->右键->属性->配置属性-->常规--->字符集---->使用多字节符字符集->确定

        If (LM_DLL <> 0) Then
            P_Init_can = GetProcAddress(LM_DLL, "Init_can")
            P_Quit_can = GetProcAddress(LM_DLL, "Quit_can")
            P_Can_send = GetProcAddress(LM_DLL, "Can_send")
            P_Can_receive = GetProcAddress(LM_DLL, "Can_receive")
            Init_can = Marshal.GetDelegateForFunctionPointer(P_Init_can, GetType(TYPE_Init_can))
            Quit_can = Marshal.GetDelegateForFunctionPointer(P_Quit_can, GetType(TYPE_Quit_can))
            Can_send = Marshal.GetDelegateForFunctionPointer(P_Can_send, GetType(TYPE_Can_send))
            Can_receive = Marshal.GetDelegateForFunctionPointer(P_Can_receive, GetType(TYPE_Can_receive))
            '需要加上声明 Imports System.Runtime.InteropServices
            TextBox1.Text += "加载DLL成功" + Environment.NewLine
        Else
            TextBox1.Text += "加载DLL失败" + Environment.NewLine
        End If
        ' //这里是动态加载 DLL函数例子。

        TextBox1.SelectionLength = 0
        TextBox1.SelectionStart = TextBox1.Text.Length
        TextBox1.ScrollToCaret()
    End Sub

    Private Sub Button2_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button2.Click
        Dim ret As Integer
        Dim RXF(4) As Byte
        Dim RXM(4) As Byte
        ret = Init_can(0, 1, 20, 0, 0, RXF, RXM) ' //调用连接A1+设备函数例子
        'ret 返回值 0：正常  1：已连接设备  2：无应答（端口有效无应答）3：无可用的串口（3-30）
        If (ret = 0) Then
            TextBox1.Text += "连接设备成功" + Environment.NewLine
        Else
            TextBox1.Text += "连接设备失败，返回值" + Str(ret) + Environment.NewLine
        End If

        TextBox1.SelectionLength = 0
        TextBox1.SelectionStart = TextBox1.Text.Length
        TextBox1.ScrollToCaret()

    End Sub




    Private Sub Button3_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button3.Click
        Dim ret As Integer
        Dim IDbuff(4) As Byte
        Dim Databuff(8) As Byte
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
        ret = Can_send(IDbuff, Databuff, FreamType, Bytes) ' //调用发送函数例子
        TextBox1.Text += "发送:" + Str(IDbuff(0)) + "  " + Str(IDbuff(1)) + "    " + Str(Databuff(0)) + "  " + Str(Databuff(1)) + "  " + Str(Databuff(2)) + "  " + Str(Databuff(3)) + "  " + Str(Databuff(4)) + "  " + Str(Databuff(5)) + "  " + Str(Databuff(6)) + "  " + Str(Databuff(7)) + Environment.NewLine
        TextBox1.SelectionLength = 0
        TextBox1.SelectionStart = TextBox1.Text.Length
        TextBox1.ScrollToCaret()
    End Sub

    Private Sub Button5_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button5.Click
        Dim ret As Integer
        ret = Quit_can() ' //调用释放函数例子
        TextBox1.Text += "释放设备连接" + Environment.NewLine
        TextBox1.SelectionLength = 0
        TextBox1.SelectionStart = TextBox1.Text.Length
        TextBox1.ScrollToCaret()
    End Sub

    Private Sub Button4_Click(ByVal sender As System.Object, ByVal e As System.EventArgs) Handles Button4.Click
        Dim ret As Integer
        Dim no_data As Integer
        Dim IDbuff(4) As Byte
        Dim Databuff(8) As Byte
        Dim FreamType(1) As Byte
        Dim Bytes(1) As Byte
        ret = 1
        no_data = 1
        Do While (ret = 1)
            ret = Can_receive(IDbuff, Databuff, FreamType, Bytes) '//调用接收函数例子
            If (ret = 1) Then
                TextBox1.Text += "接收:" + Str(IDbuff(0)) + "  " + Str(IDbuff(1)) + "    " + Str(Databuff(0)) + "  " + Str(Databuff(1)) + "  " + Str(Databuff(2)) + "  " + Str(Databuff(3)) + "  " + Str(Databuff(4)) + "  " + Str(Databuff(5)) + "  " + Str(Databuff(6)) + "  " + Str(Databuff(7)) + Environment.NewLine
                no_data = 0
            End If
        Loop
        If (no_data = 1) Then
            TextBox1.Text += "接收: 无数据" + Environment.NewLine
        End If

        TextBox1.SelectionLength = 0
        TextBox1.SelectionStart = TextBox1.Text.Length
        TextBox1.ScrollToCaret()
    End Sub
End Class
