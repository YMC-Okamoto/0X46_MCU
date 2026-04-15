#デバッガーと接続
if debugger.DebugTool.GetType() != DebugTool.Simulator:
	debugger.DebugTool.Change(DebugTool.Simulator)

if debugger.IsConnected() != True:
	debugger.Connect()

debugger.Download.LoadModule()
debugger.Reset()
debugger.Breakpoint.Delete()

#ブレーキポイントの設定
bp = BreakCondition()
bp.Address ="main"
debugger.Breakpoint.Set(bp)

#プログラム実行ブレーキ待つ
#debugger.Go(GoOption.WaitBreak)

#高速オンチップオシレータ発振安定
debugger.Register.SetValue("SYSTEM.OSCOVFSR.HCOVF", 0x01)

#メインクロック発振安定
debugger.Register.SetValue("SYSTEM.OSCOVFSR.MOOVF", 0x01)

#PLLクロック発振安定
debugger.Register.SetValue("SYSTEM.OSCOVFSR.PLOVF", 0x01)

#IWDT専用クロック発振安定
debugger.Register.SetValue("SYSTEM.OSCOVFSR.ILCOVF", 0x01)

#プログラム実行ブレーキ待つ
debugger.Go(GoOption.WaitBreak)


