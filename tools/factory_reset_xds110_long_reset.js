importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = java.lang.System.getenv("MSPM0_XDS110_CONFIG");
if (config == null || String(config).length == 0) {
    config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110_SLOW.ccxml";
}

var DSSM_BC_FACTORY_RESET = 0x020A;
var SECAP_CTL_MASK = 0xFFFF;
var SECAP_CMD_MASK = 0x00FE;
var SECAP_RX_FULL = 0x00000001;
var DSSM_CMD_RECEIVED = 0x0100;

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var cs = null;
var sec = null;
var exitCode = 1;

server.setConfig(config);

function sleep(ms) {
    java.lang.Thread.sleep(ms);
}

function evalSafe(session, expr) {
    return session.expression.evaluate(expr);
}

function connectSession(pattern) {
    var s = server.openSession(pattern);
    s.target.connect();
    return s;
}

function reconnectSecAp() {
    try {
        sec.target.disconnect();
    } catch (ignoreDisconnect) {
    }
    sleep(300);
    sec.target.connect();
}

function tryReadResponse() {
    var i;
    for (i = 0; i < 40; ++i) {
        try {
            var rcr = Number(evalSafe(cs, "'REG'::SECAP_RCR"));
            print("SECAP_RCR[" + i + "] = 0x" + (rcr >>> 0).toString(16));
            if ((rcr & SECAP_RX_FULL) == SECAP_RX_FULL) {
                var rdr = Number(evalSafe(cs, "'REG'::SECAP_RDR"));
                var rxCmd = rcr & SECAP_CMD_MASK;
                var rxResp = rdr >>> 8;
                print("SECAP_RDR = 0x" + (rdr >>> 0).toString(16));
                print("rxCmd = 0x" + rxCmd.toString(16) + ", rxResp = 0x" + rxResp.toString(16));
                if ((rxCmd == (DSSM_BC_FACTORY_RESET & SECAP_CMD_MASK)) &&
                    (rxResp == DSSM_CMD_RECEIVED)) {
                    return true;
                }
            }
        } catch (readError) {
            print("response read warning[" + i + "]: " + readError);
            try {
                reconnectSecAp();
            } catch (reconnectError) {
                print("SEC_AP reconnect warning[" + i + "]: " + reconnectError);
            }
        }
        sleep(250);
    }
    return false;
}

try {
    if (java.lang.System.getenv("MSPM0_ALLOW_FACTORY_RESET") != "YES") {
        print("Long-reset Factory Reset aborted: use wrapper with -ConfirmFactoryReset");
        exitCode = 2;
    } else {
        print("config: " + config);
        print("openSession: CS_DAP");
        cs = connectSession(".*CS_DAP.*");
        print("connected: CS_DAP");
        print("openSession: SEC_AP");
        sec = connectSession(".*SEC_AP.*");
        print("connected: SEC_AP");

        print("send DSSM Factory Reset command");
        evalSafe(cs, "'REG'::SECAP_TCR = 0x020A");
        evalSafe(cs, "'REG'::SECAP_TDR = 0");

        sleep(1000);
        print("assert NRST for long reset");
        evalSafe(sec, "GEL_AdvancedReset(\"XDS System Reset (Assert)\")");
        sleep(1500);
        print("deassert NRST");
        evalSafe(sec, "GEL_AdvancedReset(\"XDS System Reset (De-Assert)\")");
        sleep(1500);

        print("reconnect SEC_AP");
        reconnectSecAp();
        if (tryReadResponse()) {
            print("Command execution completed.");
            exitCode = 0;
        } else {
            print("Command execution failed: no valid SEC_AP response.");
            exitCode = 1;
        }
    }
} catch (error) {
    print("Long-reset Factory Reset failed: " + error);
    exitCode = 1;
} finally {
    if (sec != null) {
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
