package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.TelemetryObservable;

import java.util.ArrayList;
import java.util.List;

public enum ActionScheduler implements TelemetryObservable {
    INSTANCE;

    private List<Action> requestedActions = new ArrayList<>();
    private boolean isStopRequested = false;

    public void schedule(Action action){
        requestedActions.add(action);
    }

    public void init(){
        isStopRequested = false;
        requestedActions.clear();
    }

    public void run(){
        TelemetryPacket packet = new TelemetryPacket();

        List<Action> runningActions = new ArrayList<>();
        for (Action action : requestedActions) { // Every action in requestedActions is previewed, ran, and added to runningActions
            action.preview(packet.fieldOverlay());
            if (action.run(packet) && !isStopRequested) {
                runningActions.add(action);
            }
        }
        requestedActions = runningActions;
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void stop(){
        isStopRequested = true;
        requestedActions.clear();
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("# of actions", requestedActions.size());
        telemetry.addData("\nRequested actions", requestedActions);

    }

    @Override
    public String getName() {
        return "Action scheduler";
    }
}