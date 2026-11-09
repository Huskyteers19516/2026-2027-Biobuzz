# Panels 使用说明

Panels 是装在机器人 App 里的网页面板。电脑连上 Control Hub 的 Wi-Fi 后，用浏览器打开就能实时看 telemetry、看机器人在场地上的位置，还能直接改代码里的参数，不用重新部署。

## 一、每次使用的步骤

1. 用 Android Studio 正常把 **TeamCode**（比赛代码）部署到 Control Hub。用 Teamtestcode 也可以，两个 App 都装了 Panels，Hub 上装的是哪个就用哪个。
   - 第一次装 Panels 或升级 Panels 版本时，先 `Build → Clean Project` 再部署，避免旧的网页文件被缓存。
2. 电脑连上 Control Hub 的 Wi-Fi（和 Driver Hub 连的是同一个网络）。
3. 打开浏览器访问 **http://192.168.43.1:8001**，或者双击桌面上的 **FTC Panels** 图标。
4. 在 Driver Hub 上 INIT / START 任意 OpMode，网页上就会显示数据。

电脑上什么都不用装，浏览器就够了。

## 二、网页上能做什么

| 页面 | 作用 |
|---|---|
| Telemetry | 所有 OpMode 的 telemetry 都会自动同步过来，和 Driver Hub 上看到的一样，不用改任何 OpMode 代码 |
| Field | 在场地图上画出机器人位置和朝向（圆圈 + 朝向线） |
| Configurables | 实时修改参数（见下面第三部分） |
| OpMode Control | 直接在网页上选择、INIT、START、STOP OpMode |
| Gamepad / Battery / Graph / Capture 等 | Panels 自带的其它工具，可以自己点开看看 |

会在 Field 上画机器人的 OpMode：Robot TeleOp、Match Auto、Park - Around Rack、Park - Under Rack、Simple Park Auto，以及 Teamtestcode 里的 Pedro Pinpoint Pose Test。

## 三、实时改参数（Configurables）

在 Configurables 页面按类名找到参数，改完会立刻写进机器人内存。

**重要：网页上改的值不会保存。** 重启 App 或重新部署后会变回代码里的值。调到满意的数值以后，一定要手动把它抄回对应的 Java 文件里。

### 立刻生效（下一次循环就用新值）

| 类 | 参数 |
|---|---|
| Constants | INTAKE_POWER, TRANSFER_POWER, FLOWER_POSITIONS（只改数值，保持 3 个）, BLOCKER_CLOSED_POSITION, BLOCKER_OPEN_POSITION, SHOOTER_HEADING_OFFSET_DEGREES, AIM_KP, AIM_MIN_TURN_POWER, AIM_MAX_TURN_POWER, AIM_TOLERANCE_DEGREES, AIM_SETTLE_SECONDS, CAMERA_ROLL_OFFSET_DEGREES, GOAL_UP_MAX_ABS_ROLL_DEGREES, GOAL_OPEN_WAIT_SECONDS |
| Launcher | INTEGRAL_ZONE, INTEGRAL_LIMIT, AUTO_SHOT_TIMEOUT_SECONDS, PULSE_SPINUP_SECONDS, PULSE_OPEN_SECONDS, PULSE_RECOVER_SECONDS |
| RobotTeleOp | SLOW_SCALE, TELEOP_INTAKE_POWER, INTAKE_REVERSE_POWER, OUTTAKE_SMALL_STEP |
| MatchAuto | SETTLE_SECONDS_BEFORE_SHOOT, TURN_TOLERANCE_DEGREES |
| SimpleParkAuto | DRIVE_KP, TURN_KP, MIN_DRIVE_POWER, MAX_DRIVE_POWER, MIN_TURN_POWER, MAX_TURN_POWER, POSITION_TOLERANCE_INCHES, HEADING_TOLERANCE_DEGREES |
| ShootThenDriveAuto | DRIVE_SECONDS, DRIVE_POWER |
| WheelOdometry | FORWARD_SCALE, STRAFE_SCALE |
| GoalScanner | MIN_PERCENT_CLUSTER_FOUND, SIGHTING_FRESHNESS_MS, VIEW_MEMORY_MS |

### 下一次 INIT 才生效

这些值在 Launcher 创建时（OpMode INIT 时）被复制一次。改完以后要 STOP，再重新 INIT 一次 OpMode：

Launcher 的 DEFAULT_KS, DEFAULT_KV, DEFAULT_KP, DEFAULT_KI, RECOVERY_BOOST_ENTER, RECOVERY_BOOST_EXIT, DEFAULT_READY_TOLERANCE, DEFAULT_READY_STABLE_SECONDS, DEFAULT_SHOT_DETECT_DROP, DEFAULT_FEED_MAX_SECONDS, DEFAULT_PENDING_STAGING_POWER, DEFAULT_BLOCKER_CLOSE_SECONDS, DEFAULT_FEED_TRANSFER_POWER, DEFAULT_FEED_SETPOINT_BOOST, DEFAULT_FEED_SETPOINT_BOOST_RAMP_SECONDS, AUTO_READY_TIMEOUT_SECONDS, AUTO_STAGING_TRANSFER_POWER。

DEFAULT_TARGET_VELOCITY 两种都有：Robot TeleOp 在没有瞄准解时会实时读它，其它地方要重新 INIT。

在 Teamtestcode 的 **New Launcher Tester** 里，Launcher 的这些参数每个循环都会重新读，所以全部都是立刻生效，最适合用来调飞轮。

**Launcher Stability Test** 的 "Copy into Launcher.java" 现在和 Launcher.java 里写的原始数值比较，所以网页上改过、或者手柄调过的值都会列出来，照着抄回 Launcher.java 就行。

### 注意事项

- Panels 不检查数值范围，输错了会直接生效。不要把 MIN 设得比 MAX 大（例如 MIN_DRIVE_POWER 和 MAX_DRIVE_POWER、AIM_MIN_TURN_POWER 和 AIM_MAX_TURN_POWER），INTEGRAL_LIMIT 不能是负数。
- BLOCKER_OPEN_POSITION 是开机时根据 BLOCKER_CLOSED_POSITION 算出来的。网页上改 CLOSED 不会自动带动 OPEN，两个要分别调。
- 安全开关（各种 *_CONFIRMED、GOALS_CONFIRMED、GOAL_CHECK_CALIBRATED）故意没有放进网页，防止误点让没确认的路线跑起来。要改请改代码。
- Pedro 的路径跟随参数（Constants 里的 private 值）是建 Follower 时读一次的，网页改了也没用，所以没有放进来。

## 四、比赛时

- 没有浏览器连着的时候，Panels 几乎不占用资源，telemetry 和画图都会直接跳过，不影响比赛。
- 练习赛时不要一直开着 Panels 网页，网络不好时发送数据可能让循环变慢一点。正式比赛场地本来也不允许电脑连 Hub。
- 想彻底关闭 Panels：在 Driver Hub 上运行 **Panels → Enable/Disable Panels** 这个 OpMode，按 START 切换开/关，设置会被记住。

## 五、常见问题

- **网页打不开**：确认电脑连的是 Control Hub 的 Wi-Fi；确认 Hub 上装的是新部署的 App；看看是不是被 Enable/Disable Panels 关掉了。
- **网页显示旧的或者样子很怪**：Clean Project 后重新部署，浏览器强制刷新。
- **Telemetry 页面是空的**：要先在 Driver Hub 上 INIT 一个 OpMode，并且 OpMode 里有 telemetry 输出。
- **参数改了没反应**：看上面的表，可能是"下一次 INIT 才生效"的参数。
- **Pedro 的自动调参网页**（只在 Teamtestcode 里）还在原来的地址：http://192.168.43.1:10158 ，和 Panels 互不影响。

## 六、技术说明（给写代码的同学）

- Gradle：`TeamCode/build.gradle` 和 `Teamtestcode/build.gradle` 加了 `https://mymaven.bylazar.com/releases` 仓库和 `com.bylazar:fullpanels:1.0.20`。
- `panels/PanelsBridge.java`：开机时自动给每个 OpMode 的 telemetry 套一层 `MirroredTelemetry`。Driver Hub 上的行为完全不变，只是在有浏览器连着时额外同步一份给 Panels。
- 想在自己的 OpMode 里画机器人：`PanelsBridge.drawRobot(follower.pose());` 或 `PanelsBridge.drawRobot(x, y, headingRadians);`，用 Pedro 坐标（英寸，0~144）。传进去的值要用已经算好的位置，不要为了画图多读一次硬件。
- 想让新参数能在网页上改：在类上加 `@Configurable`（`com.bylazar.configurables.annotations.Configurable`），参数写成 `public static`（不能是 `final`），代码里每个循环直接读这个静态变量。
- 不需要在 OpMode 里再用 `PanelsTelemetry` 或 `JoinedTelemetry`，telemetry 已经自动同步了。
- 同步的是 `telemetry.addData(...)` 和 `telemetry.addLine(...)`。对返回的 Item / Line 再调用 `.addData()`、`.setValue()` 的内容只会显示在 Driver Hub，不会同步到网页。
- 不要在 OpMode 的构造函数或字段初始化里保存 `telemetry`，要在 `init()` / `runOpMode()` 里用。
- `fullpanels` 自带的 Battery 插件每 5 秒读一次 Hub 电压，开销可以忽略，不影响比赛。
