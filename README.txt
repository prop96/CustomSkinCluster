CMakeLists.txt および必要なソースコード（.h/.cpp）を用意して、以下を実行するとソリューションが生成されます：
	cmake . -B . -G "Visual Studio 17 2022"

break point を利用したデバッグを行うためには以下を行います
  - プロジェクト右クリック -> スタートアッププロジェクトに設定
  - プロジェクト右クリック -> プロパティ -> 構成プロパティ -> デバッグ
    - コマンド : maya.exe のフルパス (C:\Program Files\Autodesk\Maya2024\bin\maya.exe)
    - アタッチ : はい