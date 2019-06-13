package com.arctos6135.robotpathfinder.follower;

import com.arctos6135.robotpathfinder.core.trajectory.Moment;

abstract public class DynamicFollower<T extends Moment> extends Follower<T> {
    
    protected double updateDelay = Double.NaN;
    protected double lastUpdateTime;

    abstract public void _update();
    public void update() {
        _update();
        lastUpdateTime = timer.getTimestamp();
    }

    @Override
    public void initialize() {
        super.initialize();
        lastUpdateTime = timer.getTimestamp();
    }

    @Override
    public void run() {
        super.run();
        
        double currentTime = timer.getTimestamp();
        if(currentTime - lastUpdateTime >= updateDelay) {
            update();
            lastUpdateTime = currentTime;
        }
    }

    public double getUpdateDelay() {
        return updateDelay;
    }
    public void setUpdateDelay(double updateDelay) {
        this.updateDelay = updateDelay;
    }
}
