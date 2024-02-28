package it.unitn.rti.bdi.javaff;

public class OperationResult {

    public OperationResult(boolean result, String msg){
        this.result = result;
        this.msg = msg;
    }

    public boolean result;
    public String msg;
}
