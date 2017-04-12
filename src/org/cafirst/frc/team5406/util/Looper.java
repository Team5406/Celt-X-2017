package org.cafirst.frc.team5406.util;

import java.util.Timer;
import java.util.TimerTask;

/**
 * A Looper is an easy way to create a timed task the gets
 * called periodically.
 * <p>
 * Just make a new Looper and give it a Loopable.
 *
 * @author Tom Bottiglieri
 */
public class Looper {
  
  private Loopable loopable;
  private String m_name;
  
  private double period = 1.0 / 100.0;
  private Timer looperUpdater;
  
  public Looper(String name, Loopable loopable, double period){
    this(name, loopable, period, false);
  }
  
  public Looper(String name, Loopable loopable, double period, boolean autoStart){
    this.period = period;
    this.loopable = loopable;
    this.m_name = name;
    if(autoStart){
      this.start();
    }
  }
  
  public void start(){
    if(!isRunning()){
      looperUpdater = new Timer("Looper - " + this.m_name);
      looperUpdater.schedule(new UpdaterTask(this), 0L, (long)(this.period * 1000));
    }
  }
  
  public void stop(){
    if(isRunning()){
      looperUpdater.cancel();
      looperUpdater = null;
    }
  }
  
  public boolean isRunning(){
    return looperUpdater != null;
  }
  
  private void update(){
    loopable.update();
  }
  
  public interface Loopable {
    void update();
  }
  
  private class UpdaterTask extends TimerTask {
    
    private Looper looper;
    
    public UpdaterTask(Looper looper){
      if(looper == null){
        throw new NullPointerException("Given Looper was null");
      }
      this.looper = looper;
    }
    
    public void run(){
      looper.update();
    }
  }
}