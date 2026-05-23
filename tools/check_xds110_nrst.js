importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = java.lang.System.getenv("MSPM0_XDS110_CONFIG");
if (config == null || String(config).length == 0) {
    config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110_SLOW.ccxml";
}

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var cs = null;
var sec = null;
var exitCode = 0;

server.setConfig(config);

function sleep(ms) {
    java.lang.Thread.sleep(ms);
}

function readBootDiag(label) {
    try {
        print(label);
        cs.expression.evaluate("MSPM0_BootDiag_DebugRead()");
    } catch (error) {
        print(label + " failed: " + error);
        exitCode = 1;
    }
}

try {
    print("config: " + config);
    print("openSession: CS_DAP");
    cs = server.openSession(".*CS_DAP.*");
    cs.target.connect();
    print("connected: CS_DAP");

    print("openSession: SEC_AP");
    sec = server.openSession(".*SEC_AP.*");
    sec.target.connect();
    print("connected: SEC_AP");

    readBootDiag("BOOTDIAG before NRST assert");

    print("assert XDS System Reset");
    sec.expression.evaluate("GEL_AdvancedReset(\"XDS System Reset (Assert)\")");
    sleep(500);
    readBootDiag("BOOTDIAG while NRST asserted by XDS");

    print("deassert XDS System Reset");
    sec.expression.evaluate("GEL_AdvancedReset(\"XDS System Reset (De-Assert)\")");
    sleep(500);
    readBootDiag("BOOTDIAG after NRST deassert");
} catch (error) {
    print("NRST diagnostic failed: " + error);
    exitCode = 1;
} finally {
    if (sec != null) {
        try {
            sec.expression.evaluate("GEL_AdvancedReset(\"XDS System Reset (De-Assert)\")");
        } catch (ignoreReset) {
        }
        try {
            sec.target.disconnect();
        } catch (ignoreSecDisconnect) {
        }
        try {
            sec.terminate();
        } catch (ignoreSecTerminate) {
        }
    }
    if (cs != null) {
        try {
            cs.target.disconnect();
        } catch (ignoreCsDisconnect) {
        }
        try {
            cs.terminate();
        } catch (ignoreCsTerminate) {
        }
    }
    server.stop();
}

java.lang.System.exit(exitCode);
