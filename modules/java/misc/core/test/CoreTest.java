package org.visp.test.core;

import org.visp.core.Core;

public class CoreTest extends ViSPTestCase {

    public void testVersion() {
        assertEquals(Core.VERSION_MAJOR, Core.getVersionMajor());
        assertEquals(Core.VERSION_MINOR, Core.getVersionMinor());
        assertEquals(Core.VERSION_REVISION, Core.getVersionRevision());
        assertEquals(Core.VERSION, Core.getVersionString());
    }

}
