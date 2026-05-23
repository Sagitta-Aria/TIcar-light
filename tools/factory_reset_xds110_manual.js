importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = java.lang.System.getenv("MSPM0_XDS110_CONFIG");
if (config == null || String(config).length == 0) {
    config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110.ccxml";
}

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var session = null;
var exitCode = 0;

server.setConfig(config);

try {
    if (java.lang.System.getenv("MSPM0_ALLOW_FACTORY_RESET") != "YES") {
        print("Manual Factory Reset aborted: use the PowerShell wrapper with -ConfirmFactoryReset");
        exitCode = 2;
    } else {
        print("openSession: CS_DAP");
        session = server.openSession(".*CS_DAP.*");
        session.target.connect();
        print("connected: CS_DAP");

        print("execute DSSM Factory Reset with manual NRST");
        print("When GEL prints 'Press the reset button...', press and release board RESET once.");
        session.expression.evaluate("GEL_DAPInit_remoteFactoryReset(0)");

        java.lang.Thread.sleep(30000);
        print("Manual Factory Reset command window finished");
    }
} catch (error) {
    print("Manual Factory Reset failed: " + error);
    exitCode = 1;
} finally {
    if (session != null) {
        try {
            session.target.disconnect();
        } catch (disconnectError) {
        }
        try {
            session.terminate();
        } catch (terminateError) {
        }
    }
    server.stop();
}

java.lang.System.exit(exitCode);
