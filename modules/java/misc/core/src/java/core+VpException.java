package org.visp.core;

public class VpException extends RuntimeException {

    private static final long serialVersionUID = 1L;

    public VpException(String msg) {
        super(msg);
    }

    @Override
    public String toString() {
        return "VpException [" + super.toString() + "]";
    }
}
