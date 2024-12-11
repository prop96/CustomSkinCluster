### Direct Delta Mush Skinning

Direct Delta Mush (DDM) Skinning [1] は、詳細なウェイト調整なしでも破綻の少ないメッシュ変形が可能なリアルタイム向けスキニングアルゴリズムです。

この maya プラグインでは以下のスキニングアルゴリズムから選択してスキニング結果を比較できます：
- Linear Blend Skinning (LBS)
- LBS + Delta Mush[2]
- DDM ver0 ~ ver5 (各 ver の詳細は論文参照)


#### 結果の例：
- LBS：膝の部分が破綻
  ![LBS](https://github.com/user-attachments/assets/aeb5a95c-dab6-4bd0-b8b9-e0554a7dcb3d)
- LBS + DM：膝の部分の破綻が解消
  ![LBS+DM](https://github.com/user-attachments/assets/a281c998-49b0-428e-93fb-66d0c382dbe4)
- DDM ver0：リアルタイム向けアルゴリズムでありながら、LBS + DM とほぼ同じ結果が得られる
  ![DDM](https://github.com/user-attachments/assets/2d4c3f89-9b89-400c-aed7-40025fa95875)


[1] B. H. Le and J. P. Lewis, ACM Transactions on Graphics, Vol 38, 1 (2019): https://doi.org/10.1145/3306346.3322982

[2] J. Mancewicz, M. L. Derksen, H. Rijpkema, and C. A. Wilson, InProceedings of DigiPro ’14. ACM, New York, NY, USA, 7–11: https://doi.org/10.1145/2633374.2633376

<!--
CMakeLists.txt および必要なソースコード（.h/.cpp）を用意して、以下を実行するとソリューションが生成されます：
	cmake . -B . -G "Visual Studio 17 2022"

break point を利用したデバッグを行うためには以下を行います
  - プロジェクト右クリック -> スタートアッププロジェクトに設定
  - プロジェクト右クリック -> プロパティ -> 構成プロパティ -> デバッグ
    - コマンド : maya.exe のフルパス (C:\Program Files\Autodesk\Maya2024\bin\maya.exe)
    - アタッチ : はい
-->
