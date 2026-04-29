# vision_detection/config.py
# ================= 模型路径 =================
# 相对于包的共享目录
MODEL_BODY_FILENAME = "cylinder_best.pt"      # 阶段一：找管身
MODEL_RIM_FILENAME = "rim_best.pt"            # 阶段二：找内孔

# ================= 物理参数 =================
REAL_TUBE_DIAMETER = 0.050            # 真实管径 (米)

# ================= 裁剪与Mask =================
CROP_PADDING_RATIO = 0.25             # 裁剪小图时的外扩比例
RIM_MASK_DILATION = 15                # 内孔Mask向外扩充像素数
RIM_MASK_DILATION_MIN = 5             # 自适应内孔Mask膨胀最小值
RIM_MASK_DILATION_MAX = 25            # 自适应内孔Mask膨胀最大值
RIM_MASK_DILATION_SCALE = 0.15        # 自适应系数: dilation = bbox_w * scale
RIM_MASK_CLOSE_MIN = 3                # 第一阶段改造：rim mask 闭运算最小核
RIM_MASK_CLOSE_MAX = 11               # 第一阶段改造：rim mask 闭运算最大核
RIM_MASK_CLOSE_SCALE = 0.06           # 第一阶段改造：闭运算核随 bbox 宽度缩放
RIM_BOUNDARY_BAND_MIN = 2             # boundary band 最小半宽
RIM_BOUNDARY_BAND_MAX = 8             # boundary band 最大半宽
RIM_BOUNDARY_BAND_RADIUS_RATIO = 0.18 # boundary band 半宽相对等效半径比例

# ================= 图像处理 =================
CLAHE_CLIP = 3.0                      # 对比度增强强度
CLAHE_GRID = (4, 4)                   # CLAHE 网格大小
CANNY_SIGMA = 0.33                    # Canny 自动阈值系数

# ================= 筛选与拟合 =================
MIN_EDGE_LENGTH = 15                  # 忽略短于此像素的边缘
MIN_AXIS_RATIO = 0.20                 # 忽略太扁的椭圆
REFINEMENT_ITERATIONS = 2             # 椭圆精修迭代次数
ARC_SEGMENT_MIN_POINTS = 16           # 弧段最少点数
ARC_SEGMENT_MIN_SPAN_DEG = 28.0       # 弧段最小角跨度
ARC_SEGMENT_MAX_RESIDUAL_PX = 3.2     # 弧段拟合最大残差
CANDIDATE_INLIER_THRESH_PX = 3.0      # candidate 支持点最大残差
RIM_CANDIDATE_PAIR_TOPK = 6           # 参与配对的最长弧段数量
RIM_CANDIDATE_MAX_COUNT = 6           # 最终保留的候选数量
CANDIDATE_SCORE_MIN = 0.12            # 候选最低保留分数
HOUGH_LINE_THRESHOLD = 18             # ROI 内线段检测阈值
HOUGH_MIN_LINE_LENGTH = 18            # ROI 内线段最小长度
HOUGH_MAX_LINE_GAP = 6                # ROI 内线段最大间隙
SIDE_LINE_RESIDUAL_MAX_PX = 1.6       # 线段近似直线残差阈值
POSE_CENTER_SHIFT_GAIN = 0.18         # 双候选投影圆心修正的启发式位移系数
POSE_CENTER_SHIFT_MAX_RATIO = 0.35    # 圆心修正量相对短轴的上限比例
POSE_RGB_BODY_CENTER_WEIGHT = 0.55    # 仅 RGB 选解时，body 中心方向权重
POSE_RGB_CENTERLINE_WEIGHT = 0.25     # 仅 RGB 选解时，body 中线约束权重
POSE_RGB_BODY_AXIS_WEIGHT = 0.20      # 仅 RGB 选解时，body 主轴一致性权重
POSE_BODY_AXIS_ASPECT_MIN = 1.12      # body bbox 细长度低于此值时，方向约束置信度降低
BODY_SIDE_POINT_QUANTILE = 0.16       # 从 body 轮廓两侧各取极值点的分位数
BODY_SIDE_MIN_POINTS = 30             # 拟合侧边所需最少轮廓点
BODY_SIDE_MIN_SPAN_RATIO = 0.40       # 侧边沿 body 主方向的最小覆盖比例
BODY_SIDE_MIN_SEPARATION_RATIO = 0.18 # 两侧边最小分离比例（相对 body 短边）

# ================= 质量评估（仅诊断，不做时序门控） =================
ELLIPSE_RESIDUAL_MAX = 3.0            # 像素残差上限
RIM_COVERAGE_MIN = 0.60               # 最小边缘覆盖率
ELLIPSE_AREA_MIN_PX2 = 1200.0         # 椭圆面积下限
ELLIPSE_AREA_MAX_PX2 = 70000.0        # 椭圆面积上限
AXIS_RATIO_QUALITY_MIN = 0.15         # 轴比有效下限
AXIS_RATIO_QUALITY_MAX = 0.95         # 轴比有效上限

# ================= 深度先验融合（D435i） =================
ENABLE_DEPTH_PRIOR = False            # 第一阶段默认关闭，不纳入主估计链路
DEPTH_MIN_M = 0.15                    # 有效最小深度（米）
DEPTH_MAX_M = 1.20                    # 有效最大深度（米）
DEPTH_VALID_RATIO_MIN = 0.20          # mask内有效深度最小比例
DEPTH_IQR_MAX_M = 0.05                # 深度IQR上限（米）
DEPTH_PERCENTILE_LOW = 10.0           # 深度鲁棒截断低分位
DEPTH_PERCENTILE_HIGH = 90.0          # 深度鲁棒截断高分位
DEPTH_PRIOR_WEIGHT_MIN = 0.15         # 深度融合最小权重
DEPTH_PRIOR_WEIGHT_MAX = 0.70         # 深度融合最大权重
DEPTH_PRIOR_WEIGHT_NEAR_CIRCLE = 0.65 # 近圆退化时深度融合权重下限
DEPTH_PRIOR_MAX_DELTA_M = 0.20        # 深度先验与几何Z差值过大时降权阈值
NEAR_CIRCLE_AXIS_RATIO = 0.93         # 近似正圆判据

# ================= 法向方向约定 =================
# True: 法向强制“背离相机”（与相机->目标中心向量同向）
# False: 法向强制“朝向相机”
NORMAL_POINT_AWAY_FROM_CAMERA = True

# ================= 调试 =================
DRAW_AXES = True                      # 是否绘制坐标轴
PUBLISH_DEBUG_IMAGE = False            # 是否发布调试图像
ROI_DEBUG_SHOW_EXTRACTION_LAYERS = False  # ROI调试图默认隐藏边缘/弧段等前端细节
ROI_DEBUG_ALT_CANDIDATES = 2              # ROI调试图默认仅显示少量备选候选


# ================= Step1 重构：观测语义与候选搜索 =================
PUBLISH_ONCE_PER_SELECTION = False
SELECTED_SEMANTICS_TOPIC = "/vision/selected_cylinder_semantics"
RIM_SELECTION_CENTER_BIAS = 0.35
DEBUG_TOP_CANDIDATES = 4
