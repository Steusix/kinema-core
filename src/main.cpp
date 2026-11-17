#include "raylib.h"
#include "kinematics/PlanarArmKinematics.hpp"
#include "control/PidController.hpp"
#include <vector>
#include <deque>
#include <string>
#include <cstdio>
#include <cmath>
#include <algorithm>

struct NumberInputBox {
    Rectangle bounds;
    std::string label;
    std::string text;
    bool active = false;

    void update() {
        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            active = CheckCollisionPointRec(GetMousePosition(), bounds);
        }
        if (!active) return;

        int key = GetCharPressed();
        while (key > 0) {
            bool isDigit = (key >= '0' && key <= '9');
            bool isDot = (key == '.' && text.find('.') == std::string::npos);
            bool isMinus = (key == '-' && text.empty());
            if ((isDigit || isDot || isMinus) && text.size() < 10) {
                text.push_back(static_cast<char>(key));
            }
            key = GetCharPressed();
        }

        if (IsKeyPressed(KEY_BACKSPACE) && !text.empty()) {
            text.pop_back();
        }
    }

    void draw() const {
        DrawText(label.c_str(), static_cast<int>(bounds.x), static_cast<int>(bounds.y - 20), 16, LIGHTGRAY);
        DrawRectangleRec(bounds, active ? Color{ 45, 52, 66, 255 } : Color{ 32, 36, 46, 255 });
        DrawRectangleLinesEx(bounds, 2.0f, active ? SKYBLUE : GRAY);
        DrawText(text.c_str(), static_cast<int>(bounds.x + 10), static_cast<int>(bounds.y + 8), 20, RAYWHITE);
    }

    double parseOr(double fallback) const {
        try {
            if (text.empty() || text == "-" || text == ".") return fallback;
            return std::stod(text);
        } catch (...) {
            return fallback;
        }
    }
};

int main() {
    constexpr int screenWidth = 1280;
    constexpr int screenHeight = 820;
    constexpr int panelWidth = 370;
    constexpr int canvasWidth = screenWidth - panelWidth;

    InitWindow(screenWidth, screenHeight, "Kinema-Core | 2-DOF Planar Arm (IK + PID Lab)");
    SetTargetFPS(60);

    kinema::PlanarArmKinematics arm(0.22, 0.18);
    constexpr float pixelsPerMeter = 840.0f;
    const Vector2 origin = { canvasWidth / 2.0f, screenHeight / 2.0f + 70.0f };

    kinema::JointAngles currentAngles{ 0.5, 0.5 };
    double omega1 = 0.0;
    double omega2 = 0.0;

    bool underdampedMode = false;
    double dampingFactor = 0.935;
    kinema::PidController pid1(42.0, 5.0, 13.5);
    kinema::PidController pid2(52.0, 6.0, 15.0);

    Eigen::Vector2d targetMeters = arm.forward(currentAngles).endEffector;
    bool elbowUp = false;

    std::vector<Vector2> trail;
    constexpr size_t maxTrailPoints = 140;

    std::deque<float> errorHistory(180, 0.0f);
    float peakOvershootMm = 0.0f;
    bool passedTargetOnce = false;
    double prevSignedProj = 0.0;

    const float panelX = static_cast<float>(canvasWidth + 24);
    char initX[32], initY[32];
    std::snprintf(initX, sizeof(initX), "%.3f", targetMeters.x());
    std::snprintf(initY, sizeof(initY), "%.3f", targetMeters.y());

    NumberInputBox inputX{ { panelX, 70, 150, 36 }, "Target X [m]:", initX };
    NumberInputBox inputY{ { panelX + 170, 70, 150, 36 }, "Target Y [m]:", initY };
    const Rectangle applyBtn{ panelX, 116, 320, 34 };
    const Rectangle elbowBtn{ panelX, 158, 155, 32 };
    const Rectangle resetBtn{ panelX + 165, 158, 155, 32 };
    const Rectangle pidModeBtn{ panelX, 198, 320, 32 };

    auto applyPidPreset = [&](bool underdamped) {
        underdampedMode = underdamped;
        if (underdampedMode) {
            pid1 = kinema::PidController(65.0, 18.0, 6.5);
            pid2 = kinema::PidController(75.0, 20.0, 7.5);
            dampingFactor = 0.955;
        } else {
            pid1 = kinema::PidController(42.0, 5.0, 13.5);
            pid2 = kinema::PidController(52.0, 6.0, 15.0);
            dampingFactor = 0.935;
        }
    };

    auto syncInputsFromTarget = [&]() {
        char bufX[32], bufY[32];
        std::snprintf(bufX, sizeof(bufX), "%.3f", targetMeters.x());
        std::snprintf(bufY, sizeof(bufY), "%.3f", targetMeters.y());
        inputX.text = bufX;
        inputY.text = bufY;
    };

    auto onNewTargetSet = [&]() {
        peakOvershootMm = 0.0f;
        passedTargetOnce = false;
        prevSignedProj = 0.0;
    };

    auto applyTargetFromInputs = [&]() {
        targetMeters.x() = inputX.parseOr(targetMeters.x());
        targetMeters.y() = inputY.parseOr(targetMeters.y());
        inputX.active = false;
        inputY.active = false;
        syncInputsFromTarget();
        onNewTargetSet();
    };

    auto resetSimulation = [&]() {
        currentAngles = { 0.5, 0.5 };
        omega1 = 0.0;
        omega2 = 0.0;
        elbowUp = false;
        pid1.reset();
        pid2.reset();
        trail.clear();
        std::fill(errorHistory.begin(), errorHistory.end(), 0.0f);

        targetMeters = arm.forward(currentAngles).endEffector;
        syncInputsFromTarget();
        onNewTargetSet();
    };

    auto toScreen = [&](const Eigen::Vector2d& pos) -> Vector2 {
        return Vector2{
            origin.x + static_cast<float>(pos.x()) * pixelsPerMeter,
            origin.y - static_cast<float>(pos.y()) * pixelsPerMeter
        };
    };

    while (!WindowShouldClose()) {
        const double dt = GetFrameTime();
        Vector2 mousePx = GetMousePosition();

        inputX.update();
        inputY.update();
        if ((inputX.active || inputY.active) && IsKeyPressed(KEY_ENTER)) {
            applyTargetFromInputs();
        }

        if (!inputX.active && !inputY.active) {
            if (IsKeyPressed(KEY_SPACE)) {
                elbowUp = !elbowUp;
                onNewTargetSet();
            }
            if (IsKeyPressed(KEY_R)) resetSimulation();
            if (IsKeyPressed(KEY_P)) applyPidPreset(!underdampedMode);
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            if (CheckCollisionPointRec(mousePx, applyBtn)) applyTargetFromInputs();
            if (CheckCollisionPointRec(mousePx, elbowBtn)) {
                elbowUp = !elbowUp;
                onNewTargetSet();
            }
            if (CheckCollisionPointRec(mousePx, resetBtn)) resetSimulation();
            if (CheckCollisionPointRec(mousePx, pidModeBtn)) applyPidPreset(!underdampedMode);
        }

        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && mousePx.x < canvasWidth) {
            inputX.active = false;
            inputY.active = false;
            targetMeters = Eigen::Vector2d(
                (mousePx.x - origin.x) / pixelsPerMeter,
                (origin.y - mousePx.y) / pixelsPerMeter
            );
            syncInputsFromTarget();
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                onNewTargetSet();
            }
        }

        Eigen::Vector2d clampedTarget = arm.clampToWorkspace(targetMeters);
        const bool isClamped = (targetMeters - clampedTarget).norm() > 1e-4;

        kinema::JointAngles targetAngles = arm.inverse(targetMeters, elbowUp);

        double alpha1 = pid1.update(targetAngles.theta1, currentAngles.theta1, dt);
        double alpha2 = pid2.update(targetAngles.theta2, currentAngles.theta2, dt);

        omega1 = (omega1 + alpha1 * dt) * dampingFactor;
        omega2 = (omega2 + alpha2 * dt) * dampingFactor;
        currentAngles.theta1 += omega1 * dt;
        currentAngles.theta2 += omega2 * dt;

        kinema::ArmState currentState = arm.forward(currentAngles);
        kinema::ArmState ghostState = arm.forward(targetAngles);

        const double posErrMm = (clampedTarget - currentState.endEffector).norm() * 1000.0;
        const double angleErrSum = (targetAngles.theta1 - currentAngles.theta1) + (targetAngles.theta2 - currentAngles.theta2);
        if (prevSignedProj != 0.0 && (angleErrSum * prevSignedProj < 0.0)) {
            passedTargetOnce = true;
        }
        prevSignedProj = angleErrSum;
        if (passedTargetOnce && posErrMm > peakOvershootMm) {
            peakOvershootMm = static_cast<float>(posErrMm);
        }

        errorHistory.pop_front();
        errorHistory.push_back(static_cast<float>(posErrMm));

        Vector2 basePx = toScreen(currentState.base);
        Vector2 elbowPx = toScreen(currentState.elbow);
        Vector2 endPx = toScreen(currentState.endEffector);
        Vector2 targetPx = toScreen(clampedTarget);

        trail.push_back(endPx);
        if (trail.size() > maxTrailPoints) {
            trail.erase(trail.begin());
        }

        BeginDrawing();
        ClearBackground(Color{ 20, 22, 28, 255 });

        for (int i = -4; i <= 4; ++i) {
            float offset = i * 0.1f * pixelsPerMeter;
            DrawLineV({ origin.x + offset, 0 }, { origin.x + offset, (float)screenHeight }, Fade(GRAY, 0.12f));
            DrawLineV({ 0, origin.y + offset }, { (float)canvasWidth, origin.y + offset }, Fade(GRAY, 0.12f));
        }

        DrawCircleLinesV(basePx, static_cast<float>((arm.getL1() + arm.getL2()) * pixelsPerMeter), Fade(SKYBLUE, 0.28f));
        DrawCircleLinesV(basePx, static_cast<float>(std::abs(arm.getL1() - arm.getL2()) * pixelsPerMeter), Fade(RED, 0.28f));

        for (size_t i = 1; i < trail.size(); ++i) {
            float alpha = static_cast<float>(i) / trail.size();
            DrawLineEx(trail[i - 1], trail[i], 3.0f, Fade(GREEN, alpha * 0.75f));
        }

        DrawLineEx(basePx, toScreen(ghostState.elbow), 4.0f, Fade(GRAY, 0.25f));
        DrawLineEx(toScreen(ghostState.elbow), toScreen(ghostState.endEffector), 4.0f, Fade(GRAY, 0.25f));

        DrawLineEx(basePx, elbowPx, 10.0f, Color{ 78, 168, 222, 255 });
        DrawLineEx(elbowPx, endPx, 8.0f, Color{ 86, 207, 225, 255 });
        DrawCircleV(basePx, 12.0f, LIGHTGRAY);
        DrawCircleV(elbowPx, 9.0f, ORANGE);
        DrawCircleV(endPx, 7.0f, GREEN);

        if (isClamped) {
            DrawCircleLinesV(toScreen(targetMeters), 7.0f, Fade(RED, 0.6f));
            DrawLineV(toScreen(targetMeters), targetPx, Fade(RED, 0.5f));
        }
        DrawCircleLinesV(targetPx, 10.0f, YELLOW);

        DrawRectangle(canvasWidth, 0, panelWidth, screenHeight, Color{ 28, 31, 40, 255 });
        DrawLine(canvasWidth, 0, canvasWidth, screenHeight, Color{ 55, 62, 78, 255 });

        DrawText("CONTROL & TELEMETRY", static_cast<int>(panelX), 18, 20, SKYBLUE);

        inputX.draw();
        inputY.draw();

        DrawRectangleRec(applyBtn, Color{ 46, 125, 50, 255 });
        DrawRectangleLinesEx(applyBtn, 1.5f, GREEN);
        DrawText("APPLY TARGET [ENTER]", static_cast<int>(applyBtn.x + 56), static_cast<int>(applyBtn.y + 8), 16, RAYWHITE);

        DrawRectangleRec(elbowBtn, Color{ 45, 55, 72, 255 });
        DrawRectangleLinesEx(elbowBtn, 1.5f, SKYBLUE);
        DrawText(elbowUp ? "ELBOW: UP" : "ELBOW: DOWN", static_cast<int>(elbowBtn.x + 22), static_cast<int>(elbowBtn.y + 8), 15, RAYWHITE);

        DrawRectangleRec(resetBtn, Color{ 72, 45, 45, 255 });
        DrawRectangleLinesEx(resetBtn, 1.5f, RED);
        DrawText("RESET [R]", static_cast<int>(resetBtn.x + 38), static_cast<int>(resetBtn.y + 8), 15, RAYWHITE);

        DrawRectangleRec(pidModeBtn, underdampedMode ? Color{ 82, 58, 32, 255 } : Color{ 36, 62, 68, 255 });
        DrawRectangleLinesEx(pidModeBtn, 1.5f, underdampedMode ? ORANGE : SKYBLUE);
        DrawText(
            underdampedMode ? "PID [P]: UNDERDAMPED (OVERSHOOT)" : "PID [P]: CRITICALLY DAMPED",
            static_cast<int>(pidModeBtn.x + 18), static_cast<int>(pidModeBtn.y + 8), 15, RAYWHITE
        );

        int yCursor = 248;
        auto drawSection = [&](const char* title, const char* body, Color titleColor = SKYBLUE) {
            DrawLine(static_cast<int>(panelX), yCursor - 6, screenWidth - 24, yCursor - 6, Color{ 50, 56, 70, 255 });
            DrawText(title, static_cast<int>(panelX), yCursor, 15, titleColor);
            DrawText(body, static_cast<int>(panelX), yCursor + 20, 15, RAYWHITE);
        };

        char bufWorkspace[256];
        std::snprintf(bufWorkspace, sizeof(bufWorkspace),
            "Status:   %s\n"
            "Input:    (%.3f, %.3f) m\n"
            "Clamped:  (%.3f, %.3f) m  [r=%.2fm]",
            isClamped ? "CLAMPED TO BOUNDARY" : "REACHABLE (OK)",
            targetMeters.x(), targetMeters.y(),
            clampedTarget.x(), clampedTarget.y(), targetMeters.norm()
        );
        drawSection("1. WORKSPACE & TARGET", bufWorkspace, isClamped ? ORANGE : GREEN);
        yCursor += 92;

        char bufIK[256];
        std::snprintf(bufIK, sizeof(bufIK),
            "Theta 1*: %+7.2f deg (%+.3f rad)\n"
            "Theta 2*: %+7.2f deg (%+.3f rad)",
            targetAngles.theta1 * RAD2DEG, targetAngles.theta1,
            targetAngles.theta2 * RAD2DEG, targetAngles.theta2
        );
        drawSection("2. INVERSE KINEMATICS (TARGET)", bufIK);
        yCursor += 74;

        char bufFK[256];
        std::snprintf(bufFK, sizeof(bufFK),
            "Theta 1:  %+7.2f deg (w: %+.2f rad/s)\n"
            "Theta 2:  %+7.2f deg (w: %+.2f rad/s)\n"
            "Tip XY:   (%.3f, %.3f) m",
            currentAngles.theta1 * RAD2DEG, omega1,
            currentAngles.theta2 * RAD2DEG, omega2,
            currentState.endEffector.x(), currentState.endEffector.y()
        );
        drawSection("3. FORWARD KINEMATICS (ACTUAL)", bufFK);
        yCursor += 92;

        char bufErr[256];
        std::snprintf(bufErr, sizeof(bufErr),
            "dTheta:   (%+.2f, %+.2f) deg\n"
            "Pos Err:  %6.2f mm | Overshoot: %.2f mm",
            (targetAngles.theta1 - currentAngles.theta1) * RAD2DEG,
            (targetAngles.theta2 - currentAngles.theta2) * RAD2DEG,
            posErrMm, peakOvershootMm
        );
        drawSection("4. PID STEP RESPONSE & ERROR", bufErr, posErrMm < 1.0 ? GREEN : YELLOW);
        yCursor += 76;

        const Rectangle graphBox{ panelX, static_cast<float>(yCursor), 320.0f, 155.0f };
        DrawRectangleRec(graphBox, Color{ 18, 20, 26, 255 });
        DrawRectangleLinesEx(graphBox, 1.5f, Color{ 55, 62, 78, 255 });

        float maxScaleMm = 50.0f;
        for (float val : errorHistory) {
            if (val > maxScaleMm) maxScaleMm = val * 1.15f;
        }

        const float zeroY = graphBox.y + graphBox.height - 14.0f;
        const float usableH = graphBox.height - 28.0f;
        DrawLineV({ graphBox.x, zeroY }, { graphBox.x + graphBox.width, zeroY }, Fade(GREEN, 0.4f));
        DrawLineV({ graphBox.x, zeroY - usableH * 0.5f }, { graphBox.x + graphBox.width, zeroY - usableH * 0.5f }, Fade(GRAY, 0.2f));

        char scaleLabel[32];
        std::snprintf(scaleLabel, sizeof(scaleLabel), "%.0f mm", maxScaleMm);
        DrawText(scaleLabel, static_cast<int>(graphBox.x + 6), static_cast<int>(graphBox.y + 4), 12, GRAY);
        DrawText("0 mm (target)", static_cast<int>(graphBox.x + 6), static_cast<int>(zeroY - 14), 12, Fade(GREEN, 0.7f));

        const float stepX = graphBox.width / static_cast<float>(errorHistory.size() - 1);
        for (size_t i = 1; i < errorHistory.size(); ++i) {
            float y1 = zeroY - std::min(errorHistory[i - 1] / maxScaleMm, 1.0f) * usableH;
            float y2 = zeroY - std::min(errorHistory[i] / maxScaleMm, 1.0f) * usableH;
            Color lineCol = underdampedMode ? ORANGE : SKYBLUE;
            DrawLineEx(
                { graphBox.x + (i - 1) * stepX, y1 },
                { graphBox.x + i * stepX, y2 },
                2.2f, lineCol
            );
        }

        EndDrawing();
    }

    CloseWindow();
    return 0;
}