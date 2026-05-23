importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = java.lang.System.getenv("MSPM0_XDS110_CONFIG");
if (config == null || String(config).length == 0) {
    config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110_SLOW.ccxml";
}

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var session = null;
var exitCode = 0;

server.setConfig(config);

try {
    print("config: " + config);
    print("openSession: CS_DAP");
    session = server.openSession(".*CS_DAP.*");
    session.target.connect();
    print("connected: CS_DAP");

    print("execute DSSM Wait For Debug with automatic NRST");
    session.expression.evaluate("GEL_DAPInit_remoteWaitForDebug(1)");

    java.lang.Thread.sleep(30000);
    print("Wait For Debug command window finished");
} catch (error) {
    print("Wait For Debug failed: " + error);
    exitCode = 1;
} finally {
    if (session != null) {
        try {
            session.target.disconnect();
        } catch (ignoreDisconnect) {
        }
        try {
            session.terminate();
        } catch (ignoreTerminate) {
        }
    }
    server.stop();
}

java.lang.System.exit(exitCode);
