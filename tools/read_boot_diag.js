importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110.ccxml";

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");
var session = null;

server.setConfig(config);

try {
    print("openSession: CS_DAP");
    session = server.openSession(".*CS_DAP.*");
    try {
        session.target.connect();
        print("connected: CS_DAP");
    } catch (connectError) {
        print("connect warning: " + connectError);
    }

    print("read BOOTDIAG via TI GEL helper");
    session.expression.evaluate("MSPM0_BootDiag_DebugRead()");
} catch (error) {
    print("BOOTDIAG failed: " + error);
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
