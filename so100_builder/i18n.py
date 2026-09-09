"""UI translations. BLENDER_ADDON_PLAN.md Sec 10 (2026-08-24, user request:
"is it possible to add a second language to the addon GUI?").

Blender's own localization mechanism (``bpy.app.translations``), not
anything this addon invents: every panel label, button, checkbox, dropdown
option and tooltip is plain text passed to ``bpy.types.UILayout``/``bpy.
props.*`` calls, and Blender's C-level UI drawing code already runs every
one of those strings through its own translation lookup automatically --
IF a translation for that exact string is registered. This module is only
that registration: a dict of ``{locale: {(msgctxt, english_text):
translated_text}}``, handed to ``bpy.app.translations.register()`` once
at addon startup.

**No reload needed to switch languages.** Once this registers (which
happens automatically when the addon itself loads/reloads, same as any
other code change), switching languages is instant and live: Preferences >
Interface > Translation > Language, with "Affect" > Interface/Tooltips
ticked. Blender re-draws every panel in the new language on the very next
redraw -- there is no separate per-addon language reload step. Confirmed
directly against this exact Blender build (5.2.0 LTS) by registering a
probe dict, setting ``bpy.context.preferences.view.language = "ja_JP"``,
and reading back translated strings via ``bpy.app.translations.pgettext()``
-- see the commit message / STATUS.md entry for the exact script.

**Scope, deliberately (Japanese only, static UI text only):**

* Panel titles, box/section headers, button labels, checkbox/dropdown
  labels, property names, dropdown option names, and operator/property
  tooltips are all translated below -- this is the vast majority of what a
  user actually SEES scanning the UI.
* **Not translated: anything built with runtime data** (``"%d stick(s)
  impossible"``, ``"Design:   %s" % dims``, every ``self.report(...)``
  message, every validation/error reason string). Blender's translation
  lookup is an EXACT STRING MATCH on the msgid -- by the time a Python
  ``%``-formatted string reaches ``layout.label(text=...)``, the runtime
  value is already baked in, so there is no fixed string to register a
  translation against. Making these translatable too would mean wrapping
  every format call in ``ui/panels.py`` with an explicit
  ``bpy.app.translations.pgettext_iface()`` on the TEMPLATE before
  substitution -- a real, larger refactor, not attempted here.
* **Not translated: anything in ``core/``.** Those modules stay pure
  Python with no ``bpy`` import at all (constraint B4) precisely so they
  are testable outside Blender -- adding `bpy.app.translations` calls
  there would violate that boundary for a UI concern that belongs in
  ``ui/``/``ops/`` anyway. Validation "reason" strings surfaced from
  ``core/validate.py`` therefore stay English regardless of language.
* Everything else in the codebase -- comments, docstrings, variable/
  function names, commit messages -- stays English, the project's own
  primary language, per the user's own instruction.

Only Japanese (``ja_JP``) exists today; the dict shape supports adding more
locales as additional top-level keys without touching anything else.
"""

import bpy
from bpy.app.translations import contexts as i18n_contexts

_D = i18n_contexts.default          # "*"  -- labels, checkboxes, tooltips
_OP = i18n_contexts.operator_default  # "Operator" -- operator bl_label/bl_description

_JA = {
    # --- properties.py: SO100StickItem ---------------------------------
    (_D, "ID"): "ID",
    (_D, "Stable identifier -- never reused"): "安定した識別子 -- 再利用されません",
    (_D, "Order"): "順序",
    (_D, "Build order index; -1 until an order is computed"):
        "組み立て順序のインデックス。順序が計算されるまでは -1",
    (_D, "Status"): "ステータス",
    (_D, "Reason"): "理由",
    (_D, "Specific, actionable explanation for impossible/failed"):
        "構築不可能・失敗の具体的な理由",
    (_D, "Stick Length"): "スティック長",
    (_D, "PHYSICAL length to cut -- the design edge length, after any "
        "stock snapping. This is what the human cuts and loads"):
        "実際に切断する長さ -- ストック長へのスナップ後のデザインエッジ長。"
        "人間が切断して装填する長さです",
    (_D, "Expanded Edge"): "展開後エッジ長",
    (_D, "Solved edge length after mesh expansion -- longer than the "
        "stick by the joint gaps, and NOT what gets cut"):
        "メッシュ展開後に解かれたエッジ長 -- ジョイントの隙間の分だけ"
        "スティックより長く、切断する長さではありません",
    (_D, "Residual"): "残差",
    (_D, "How far the solved edge missed its required length"):
        "解かれたエッジが必要な長さからどれだけ外れたか",
    (_D, "Shared Ends"): "共有端点数",
    (_D, "Flip"): "反転",
    (_D, "Manual override of which end is the base (the end that seats down)"):
        "どちらの端をベース（下に設置する端）にするかの手動指定",
    (_D, "Warnings"): "警告",
    (_D, "Comma-separated warning codes"): "カンマ区切りの警告コード",

    # --- properties.py: SO100WarningItem -------------------------------
    (_D, "Stick"): "スティック",
    (_D, "Code"): "コード",
    (_D, "Message"): "メッセージ",
    (_D, "Is Error"): "エラーか",

    # --- properties.py: SO100SceneProps --------------------------------
    (_D, "Robot"): "ロボット",
    (_D, "Which robot this design targets. Stamped into the "
        "exported build file as `robot` (BRIDGE_PROTOCOL.md Sec "
        "A.1.1/A.2) -- the executor refuses to run a file meant "
        "for a different arm. Selecting a robot whose kinematics "
        "package isn't vendored yet degrades cleanly: validation, "
        "the mirror rig and Export Build File all report a clear "
        "reason instead of silently using the wrong arm's geometry"):
        "このデザインの対象ロボット。書き出すビルドファイルに `robot` として"
        "記録されます（BRIDGE_PROTOCOL.md セクション A.1.1/A.2）-- "
        "実行側は対象が異なるロボット用のファイルの実行を拒否します。"
        "キネマティクスパッケージが未導入のロボットを選んだ場合も、"
        "検証・ミラーリグ・ビルドファイル書き出しはすべて、誤ったジオメトリを"
        "黙って使う代わりに明確な理由を報告します",
    (_D, "SO-100 (5-DOF)"): "SO-100（5軸）",
    (_D, "so_arm_100_kinematics -- testing/dev rig. Closed-form IK, "
        "vendored and hardware-validated"):
        "so_arm_100_kinematics -- テスト・開発用リグ。閉形式IK、"
        "導入済みで実機検証済み",
    (_D, "KUKA KR10 R900-2 (6-DOF)"): "KUKA KR10 R900-2（6軸）",
    (_D, "kr10_r900_2_kinematics -- production rig. Vendored 2026-08-22, "
        "genuine 6-DOF closed-form IK"):
        "kr10_r900_2_kinematics -- 本番用リグ。2026-08-22に導入、"
        "真の6軸閉形式IK",
    (_D, "Robot Base"): "ロボットベース",
    (_D, "An Empty marking the selected robot's own URDF root frame "
        "(`base_link` for so_arm_100, `base` for kr10_r900_2 -- see "
        "each kinematics package's own README). Moving it repositions "
        "the whole design relative to the robot with no re-authoring"):
        "選択中ロボットのURDFルートフレームを示すエンプティ"
        "（so_arm_100は `base_link`、kr10_r900_2は `base` -- 詳細は各"
        "キネマティクスパッケージのREADMEを参照）。これを移動すると、"
        "作り直すことなくロボットに対するデザイン全体の位置が変わります",
    (_D, "Design Mesh"): "デザインメッシュ",
    (_D, "The wireframe mesh. Every edge becomes one wooden stick"):
        "ワイヤーフレームメッシュ。各エッジが1本のスティックになります",
    (_D, "Build Mesh"): "ビルドメッシュ",
    (_D, "Derived, expanded mesh generated by the addon. Never edit it "
        "by hand -- it is regenerated on every extraction"):
        "アドオンが生成する展開後メッシュ。手動で編集しないでください -- "
        "抽出のたびに再生成されます",
    (_D, "Stock Section"): "ストック断面",
    (_D, "Square stock cross-section. 6.45 mm is the project's stock"):
        "角材の断面寸法。このプロジェクトのストックは6.45mmです",
    (_D, "Joint Allowance"): "ジョイント許容量",
    (_D, "How far each stick stops short of a shared vertex. The default "
        "is half the stock section, which is exact for a 90 degree "
        "joint. Raise it if shallow joints are warned about"):
        "各スティックが共有頂点の手前でどれだけ止まるか。既定値はストック"
        "断面の半分で、90度の接合には正確です。浅い角度の接合で警告が出る"
        "場合は増やしてください",
    (_D, "Length Mode"): "長さモード",
    (_D, "Design-driven"): "デザイン駆動",
    (_D, "Stick length = the edge length as drawn. Maximum design freedom, "
        "every stick potentially unique"):
        "スティック長 = 描画したエッジの長さ。設計の自由度が最大で、"
        "スティックごとに長さが異なる可能性があります",
    (_D, "Fixed stock lengths"): "固定ストック長",
    (_D, "Snap each edge to the nearest available stock length BEFORE "
        "expanding. The whole build then uses a handful of pre-cut lengths"):
        "展開前に各エッジを最も近い利用可能なストック長にスナップします。"
        "構築全体で数種類の切り出し済みの長さのみを使用します",
    (_D, "Stock Lengths"): "ストック長さ",
    (_D, "Comma-separated lengths available at the saw"):
        "のこぎりで用意できる長さ（カンマ区切り）",
    (_D, "Growth"): "成長方式",
    (_D, "Per-edge"): "エッジごと",
    (_D, "required_edge = stick + allowance x shared_ends. Free ends land "
        "exactly on the design vertex"):
        "必要エッジ長 = スティック長 + 許容量 x 共有端点数。自由端は"
        "デザインの頂点に正確に一致します",
    (_D, "Uniform"): "均一",
    (_D, "Allowance at every end, so every edge grows by exactly 2x the "
        "allowance. Free ends overhang harmlessly; far better conditioned "
        "on looped structures"):
        "すべての端に許容量を適用し、各エッジはちょうど許容量の2倍だけ"
        "伸びます。自由端は無害にはみ出します。ループ構造では条件が"
        "はるかに安定します",
    (_D, "Grounded Vertices"): "接地頂点",
    (_D, "Slide on the plate"): "プレート上をスライド",
    (_D, "Grounded vertices stay at z=0 -- nothing expands below the physical "
        "base plate -- but may slide across it as the design grows. Solves "
        "flat first layers exactly"):
        "接地頂点はz=0を保ちます -- 物理的なベースプレートより下へは展開"
        "されませんが、デザインが成長するにつれてプレート上を横に"
        "スライドすることはできます。平らな第一層を正確に解きます",
    (_D, "Pin"): "固定",
    (_D, "Grounded vertices are fully immobile. A first layer that forms a "
        "closed ring cannot expand at all under this mode and every one of "
        "its edges reports as impossible"):
        "接地頂点を完全に固定します。このモードでは閉じたリング状の第一層は"
        "まったく展開できず、そのすべてのエッジが構築不可能と報告されます",
    (_D, "Require Build Plate"): "ビルドプレートを要求",
    (_D, "Uncheck for a design held by something this addon does not "
        "model at all -- a stick's own base used as a jig, a non-flat "
        "fixture -- rather than by a flat plate at any height. No "
        "vertex is then checked against a plate, nothing is ever "
        "reported as floating, and each disconnected part of the "
        "design starts from an arbitrary point instead of a grounded "
        "one. Does not verify the result is physically self-"
        "supporting -- you are responsible for how it is actually "
        "held during the build"):
        "このアドオンが一切モデル化していない方法で保持されるデザイン"
        "（治具として使うスティックの端や、平らでない固定具など）の場合は"
        "オフにしてください -- 平らなプレートではありません。頂点はプレート"
        "に対して一切チェックされず、「浮いている」という報告も出ません。"
        "デザインの各独立部分は接地点ではなく任意の点から開始します。"
        "結果が物理的に自立することは検証されません -- 実際にどう支えるかは"
        "ユーザーの責任です",
    (_D, "Min Stick Length"): "最小スティック長",
    (_D, "Stock threshold. Checked against the SELECTED robot's own "
        "hard physical floor (min grasp offset + jaw contact "
        "half-length) at extraction time -- below that, no offset "
        "both fits within the stick and clears the floor-clearance "
        "floor"):
        "ストックのしきい値。抽出時に選択中ロボット固有の物理的な下限"
        "（最小把持オフセット + ジョー接触半長）と照合されます -- それを"
        "下回ると、スティック内に収まりかつ床面クリアランスも確保できる"
        "オフセットが存在しません",
    (_D, "Max Stick Length"): "最大スティック長",
    (_D, "Merge Distance"): "統合距離",
    (_D, "Vertices closer than this are the same structural joint"):
        "この距離より近い頂点は同一の構造的接合点とみなされます",
    (_D, "Residual Tolerance"): "残差許容値",
    (_D, "Per-edge length error the solver may leave before the edge "
        "is reported as one the sticks will not physically fit"):
        "スティックが物理的に収まらないと報告される前に、ソルバーが"
        "各エッジに残せる長さの誤差",
    (_D, "Ground Tolerance"): "接地許容値",
    (_D, "A vertex within this distance of the build plate seats on it"):
        "ビルドプレートからこの距離以内にある頂点は接地しているとみなされます",
    (_D, "Layer Height Tolerance"): "レイヤー高さ許容値",
    (_D, "Sticks whose tops are within this of each other form one "
        "layer of the build order, and the robot finishes a layer "
        "before starting the next. Within a layer it works from "
        "the far side toward itself, so it never builds a wall "
        "between its own shoulder and the sticks it still has to "
        "place. Set this below the height of one course of the "
        "design and above the sub-millimetre spread that mesh "
        "expansion leaves behind -- the default suits any design "
        "whose courses are more than a centimetre apart"):
        "上端どうしがこの範囲内にあるスティックは組み立て順序上の同一レイヤーと"
        "みなされ、ロボットは次のレイヤーに進む前に現在のレイヤーを完成させます。"
        "レイヤー内では遠い側から手前側へ向かって作業するため、まだ設置すべき"
        "スティックと自身の肩との間に壁を作ってしまうことがありません。デザイン"
        "1段分の高さより小さく、かつメッシュ展開が残すサブミリメートル単位の"
        "ばらつきより大きい値を設定してください -- 既定値は、各段が1センチ"
        "メートル以上離れているデザインであれば適合します",
    (_D, "Build Plate Height"): "ビルドプレート高さ",
    (_D, "How far the physical build plate is raised ABOVE the "
        "selected robot's own confirmed build-volume floor -- "
        "0 (default) is that floor itself, which is where the "
        "viewport build-volume box already starts (Z=0 for "
        "so_arm_100, -20mm for kr10_r900_2's own mounting "
        "pedestal). Moving this moves that box's bottom face by "
        "the same amount -- the two are always the same value. "
        "A design that sits entirely above the plate's current "
        "height -- reported as a floating component -- is not "
        "necessarily unbuildable, just unbuildable at THIS "
        "height. Raise this to the component's own lowest point "
        "(named in the error), or use Drop to Build Plate to "
        "move the design mesh to the plate instead"):
        "実際のビルドプレートが、選択中ロボット自身の確定済みビルドボリューム"
        "の床面からどれだけ上げられているか -- 既定値の0はその床面自体を"
        "表し、これはビューポートのビルドボリュームボックスが既に開始して"
        "いる位置と同じです（so_arm_100はZ=0、kr10_r900_2は自身の取り付け"
        "台の分だけZ=-20mm）。この値を変更すると、そのボックスの底面も"
        "同じ量だけ動きます -- 両者は常に同じ値です。プレートの現在の高さ"
        "より完全に上にあるデザイン（「浮いている部品」として報告される）は"
        "必ずしも構築不可能ではなく、この高さでは構築できないだけです。"
        "この値をその部品自身の最下点（エラーメッセージに記載）まで上げる"
        "か、「ビルドプレートに落とす」を使ってデザインメッシュをプレートへ"
        "移動してください",
    (_D, "Show Overlay"): "オーバーレイを表示",
    (_D, "GPU viewport overlay: build volume + per-stick status colours "
        "(Sec 10.4). Hard off switch -- kept in its own module"):
        "GPUビューポートオーバーレイ：ビルドボリュームとスティックごとの"
        "ステータス色（セクション10.4）。完全にオフにできるスイッチで、"
        "独立したモジュールに保持されています",
    (_D, "Highlight Previous Sticks"): "以前のスティックをハイライト",
    (_D, "Check By Eye also highlights every stick that comes before "
        "the current one -- in build order once one is computed, "
        "otherwise extraction order -- so the path already built up "
        "to this point is visible at a glance. Off by default: only "
        "the current stick is highlighted, same as before this "
        "existed"):
        "目視確認で、現在のスティックより前のすべてのスティックも"
        "ハイライトします -- 組み立て順序が計算済みならその順序で、"
        "そうでなければ抽出順序で判定します -- これにより、この時点までに"
        "既に組み立てた経路を一目で確認できます。既定ではオフで、この機能が"
        "追加される前と同様に現在のスティックのみがハイライトされます",
    (_D, "Show Robot Mirror"): "ロボットミラーを表示",
    (_D, "A rig posed by the vendored FK, showing the arm at the "
        "current build-order position -- a printer-style preview "
        "of the whole job before anything moves"):
        "導入済みのFKで姿勢を計算するリグで、現在の組み立て順序の位置に"
        "おけるアーム姿勢を表示します -- 実際に動く前に全体をプリンター"
        "のようにプレビューできます",
    (_D, "Sort by Build Order"): "組み立て順序で並べ替え",
    (_D, "List sticks in the order the robot will place them, "
        "rather than in extraction order"):
        "抽出順ではなく、ロボットが設置する順序でスティックを一覧表示します",
    (_D, "Backtrack Limit"): "バックトラック上限",
    (_D, "How hard the order solver may search before reporting "
        "honestly that it could not find a valid order"):
        "有効な順序が見つからなかったと正直に報告するまでに、順序ソルバーが"
        "どれだけ探索を続けてよいか",
    (_D, "Check Jaw Clearance"): "ジョー干渉チェック",
    (_D, "Check the grip region against already-placed sticks "
        "(Sec 6 C3). The jaw envelope is an ESTIMATE until Phase 0 "
        "measures it, so this warns rather than blocks"):
        "把持領域を既に設置済みのスティックと照合します（セクション6 "
        "C3）。ジョーの外形寸法はフェーズ0で実測されるまでは推定値のため、"
        "これはブロックではなく警告として扱われます",
    (_D, "Jaw Width"): "ジョー幅",
    (_D, "How far across the jaws are. ESTIMATE -- Phase 0 measures "
        "the real envelope"):
        "ジョーの開き幅。推定値です -- フェーズ0で実際の外形寸法を実測します",
    (_D, "Build File"): "ビルドファイル",
    (_D, "The exported build file. Its status sidecar sits next to "
        "it and is what ROS2 writes progress into"):
        "書き出したビルドファイル。同じ場所にステータスサイドカーが置かれ、"
        "ROS2はそこに進捗を書き込みます",
    (_D, "Advanced"): "詳細設定",

    # --- ops/design.py ---------------------------------------------------
    (_OP, "Drop to Build Plate"): "ビルドプレートに落とす",
    (_OP, "Move the design mesh so its lowest vertex touches the "
         "build plate's own current height (Build Plate Height)"):
        "デザインメッシュの最下点がビルドプレートの現在の高さ"
        "（ビルドプレート高さ）に接するように移動します",
    (_OP, "Create Robot Base"): "ロボットベースを作成",
    (_OP, "Add an Empty marking the selected robot's own URDF root "
         "frame and point the addon at it. Move it to reposition "
         "the whole design relative to the robot"):
        "選択中ロボットのURDFルートフレームを示すエンプティを追加し、"
        "アドオンをそれに向けます。移動するとロボットに対するデザイン"
        "全体の位置を変更できます",
    (_OP, "Reset Stock to This Robot's Defaults"): "このロボットの既定値にストックをリセット",
    (_OP, "Set Stock Section, Joint Allowance and Min/Max Stick "
         "Length to the selected robot's own defaults -- "
         "these fields do not switch automatically, so pressing "
         "this after changing Robot is worth doing before designing"):
        "ストック断面・ジョイント許容量・最小/最大スティック長を選択中"
        "ロボットの既定値に設定します -- これらの項目は自動では切り替わら"
        "ないため、ロボット変更後は設計を始める前に押しておく価値があります",
    (_D, "Generate Build Mesh"): "ビルドメッシュを生成",
    (_D, "Create/refresh a separate object showing the expanded, "
        "physically-inset sticks. The design mesh is never modified"):
        "展開後の物理的にインセットされたスティックを示す別オブジェクトを"
        "作成・更新します。デザインメッシュは一切変更されません",
    (_OP, "Extract Sticks"): "スティックを抽出",
    (_OP, "Turn the design mesh's edges into physical sticks: snap "
         "lengths, grow the design so fixed-length sticks fit with a "
         "glue gap at every joint, and report what will not fit"):
        "デザインメッシュのエッジを実際のスティックに変換します：長さを"
        "スナップし、固定長のスティックが各接合部で接着の隙間を持って"
        "収まるようデザインを成長させ、収まらないものを報告します",
    (_OP, "Select && Frame in Viewport"): "選択してビューポートにフレーム",
    (_OP, "Select this stick's edge on the build mesh and frame it "
         "in the 3D view"):
        "ビルドメッシュ上でこのスティックのエッジを選択し、3Dビューで"
        "フレーム表示します",
    (_OP, "Step Stick"): "スティックを移動",
    (_OP, "Clear Results"): "結果をクリア",
    (_OP, "Discard the extracted sticks and their state, and delete "
         "the build mesh -- Extract Sticks regenerates a fresh one"):
        "抽出したスティックとその状態を破棄し、ビルドメッシュも削除します"
        " -- スティック抽出を実行すると新しいものが生成されます",

    # --- ops/order.py ------------------------------------------------------
    (_OP, "Compute Build Order"): "組み立て順序を計算",
    (_OP, "Work out which stick the robot places first, second, ... "
         "so it never has to reach into a space it has already "
         "walled off. Validates each placement against the "
         "already-built structure as it goes"):
        "ロボットがどのスティックを1番目、2番目…に設置するかを決定し、"
        "既に自分で囲んでしまった空間に手を伸ばす必要がないようにします。"
        "進行しながら、既に構築済みの構造に対して各設置を検証します",
    (_OP, "Clear Build Order"): "組み立て順序をクリア",
    (_OP, "Discard the computed build order, keeping the sticks"):
        "計算済みの組み立て順序を破棄し、スティックは保持します",
    (_OP, "Move Build Step"): "組み立てステップを移動",
    (_OP, "Swap this stick with its neighbour earlier/later in the "
         "build order, then re-validate the new order"):
        "組み立て順序の中でこのスティックを前後の隣接ステップと入れ替え、"
        "新しい順序を再検証します",

    # --- ops/build.py ------------------------------------------------------
    (_OP, "Export Build File"): "ビルドファイルを書き出し",
    (_OP, "Write the build file ROS2 executes. Everything needed is "
         "in the file -- metres in base_link, already in build "
         "order, with the validation verdicts"):
        "ROS2が実行するビルドファイルを書き出します。必要な情報はすべて"
        "ファイルに含まれます -- base_link基準のメートル単位、組み立て"
        "順序済み、検証結果付き",
    (_OP, "Sync Status From Sidecar"): "サイドカーからステータスを同期",
    (_OP, "Read back what the robot actually placed. Where the "
         ".blend and the sidecar disagree, both are reported and "
         "nothing is changed for those sticks"):
        "ロボットが実際に設置した内容を読み込みます。.blendとサイドカーが"
        "食い違う場合は両方を報告し、該当するスティックは変更しません",
    (_OP, "Mark Stick"): "スティックをマーク",
    (_OP, "Record what actually happened to this stick"):
        "このスティックに実際に起きたことを記録します",
    (_D, "Placed"): "設置済み",
    (_D, "Glued in place"): "接着して設置済み",
    (_D, "Glued in place on the real sculpture"): "実物の構造物に接着して設置済み",
    (_D, "Failed"): "失敗",
    (_D, "The robot could not place it"): "ロボットが設置できませんでした",
    (_D, "Skipped"): "スキップ",
    (_D, "Deliberately passed over"): "意図的に見送りました",
    (_D, "Pending"): "未検証",
    (_D, "Not yet built"): "まだ構築されていません",
    (_D, "Not yet validated"): "まだ検証されていません",
    (_D, "Buildable"): "構築可能",
    (_D, "Validated and reachable"): "検証済みで到達可能",
    (_D, "Impossible"): "構築不可能",
    (_D, "Cannot be built -- see the reason"): "構築できません -- 理由を確認してください",
    (_OP, "Write Status Sidecar"): "ステータスサイドカーを書き出し",
    (_OP, "Write the current placed/failed state next to the build "
         "file, so ROS2 can resume from it"):
        "現在の設置済み/失敗の状態をビルドファイルの隣に書き出し、"
        "ROS2がそこから再開できるようにします",
    (_OP, "Reset Build Progress"): "ビルド進捗をリセット",
    (_OP, "Set every stick back to its pre-build state, keeping the "
         "sticks and the order"):
        "すべてのスティックを構築前の状態に戻します。スティックと順序は"
        "保持されます",
    (_OP, "Export Cut List"): "切断リストを書き出し",
    (_OP, "Write the ordered cut list a human needs at the saw and "
         "at the feeder"):
        "のこぎりとフィーダーで人間が必要とする、順序付きの切断リストを"
        "書き出します",

    # --- ops/mirror.py -------------------------------------------------
    (_OP, "Toggle Robot Mirror"): "ロボットミラーの表示切替",
    (_OP, "Step Robot Mirror"): "ロボットミラーを移動",

    # --- ui/panels.py: panel titles ------------------------------------
    (_D, "Design"): "デザイン",
    (_D, "Summary"): "サマリー",
    (_D, "Plan"): "プラン",
    (_D, "Build"): "ビルド",
    (_D, "Preview"): "プレビュー",
    (_D, "Sticks"): "スティック",
    (_D, "Reference"): "リファレンス",

    # --- ui/panels.py: static box/section labels ------------------------
    (_D, "Not vendored yet -- validation/export will report why"):
        "まだ導入されていません -- 検証・書き出し時に理由が報告されます",
    (_D, "Stock"): "ストック",
    (_D, "Mesh Expansion"): "メッシュ展開",
    (_D, "Sticks keep their length; the design grows."):
        "スティックの長さは変わらず、デザインの方が成長します。",
    (_D, "No plate check -- you supply the support"):
        "プレートチェックなし -- 支持はユーザーが用意します",
    (_D, "Dimensions"): "寸法",
    (_D, "No build order yet."): "まだ組み立て順序がありません。",
    (_D, "Errors"): "エラー",
    (_D, "Warnings (Sec 6.2)"): "警告（セクション6.2）",
    (_D, "Compute a build order first."): "先に組み立て順序を計算してください。",
    (_D, "Next to load"): "次に装填するもの",
    (_D, "Build complete."): "ビルド完了。",
    (_OP, "Placed"): "設置済み",
    (_OP, "Failed"): "失敗",
    (_OP, "Skip"): "スキップ",
    (_D, "Skip"): "スキップ",
    (_D, "Sidecar disagrees with this .blend"): "サイドカーがこの.blendと一致しません",
    (_D, "Nothing was changed for those sticks."):
        "該当するスティックは変更されていません。",
    (_D, "Hide Robot Mirror"): "ロボットミラーを隠す",
    (_D, "Robot base box (viewport reference only):"):
        "ロボットベースボックス（ビューポート参照専用）：",
    (_D, "Not confirmed for this robot yet"): "このロボットではまだ確定していません",
    (_D, "98% reachable for vertical sticks."): "垂直スティックの98%が到達可能です。",

    # --- ui/panels.py: sticks list detail box static labels -------------
    (_D, "Move Earlier"): "早める",
    (_D, "Move Later"): "遅らせる",
    (_D, "Check By Eye"): "目視確認",
}

TRANSLATIONS_DICT = {"ja_JP": _JA}


def register():
    bpy.app.translations.register(__name__, TRANSLATIONS_DICT)


def unregister():
    bpy.app.translations.unregister(__name__)
