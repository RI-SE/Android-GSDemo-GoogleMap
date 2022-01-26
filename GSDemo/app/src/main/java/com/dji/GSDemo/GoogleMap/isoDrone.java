package com.dji.GSDemo.GoogleMap;

import com.isoObject.*;

public class isoDrone extends TestObject {

    public isoDrone(long cPtr, boolean cMemoryOwn) {
        super(cPtr, cMemoryOwn);
    }

    @Override
    public void handleAbort() {
        super.handleAbort();
        //pls_land.exe
    }
}
