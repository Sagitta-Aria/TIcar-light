importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110.ccxml";

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var session = null;
var exitCode = 0;

server.setConfig(config);

try {
    if (java.lang.System.getenv("MSPM0_ALLOW_FACTORY_RESET") != "YES") {
        print("Factory Reset aborted: run tools/factory_reset_xds110.ps1 -ConfirmFactoryReset");
        exitCode = 2;
    } else {
        print("openSession: CS_DAP");
        session = server.openSession(".*CS_DAP.*");
        session.target.connect();
        print("connected: CS_DAP");

        print("execute DSSM Factory Reset with automatic NRST");
        session.expression.evaluate("GEL_DAPInit_remoteFactoryReset(1)");

        java.lang.Thread.sleep(8000);
        print("Factory Reset command window finished");
    }
} catch (error) {
    print("Factory Reset failed: " + error);
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
