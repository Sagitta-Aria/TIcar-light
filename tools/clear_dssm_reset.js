importPackage(Packages.com.ti.debug.engine.scripting);
importPackage(Packages.com.ti.ccstudio.scripting.environment);

var config = "D:/Ti/light-car1.0ccs/targetConfigs/MSPM0G3507_XDS110.ccxml";

var env = ScriptingEnvironment.instance();
var server = env.getServer("DebugServer.1");

server.setConfig(config);

function trySession(name) {
    var session = null;
    try {
        print("openSession: " + name);
        session = server.openSession(name);
        try {
            session.target.connect();
            print("connected: " + name);
        } catch (connectError) {
            print("connect warning: " + connectError);
        }

        try {
            print("write PWRAP_DPREC0 clear/reset bit");
            session.expression.evaluate("PWRAP_DPREC0 = PWRAP_DPREC0 | 0x00020000");
        } catch (evalError) {
            print("eval warning: " + evalError);
        }

        try {
            session.target.reset();
        } catch (resetError) {
            print("reset warning: " + resetError);
        }

        try {
            session.target.disconnect();
        } catch (disconnectError) {
        }
        session.terminate();
        return true;
    } catch (sessionError) {
        print("session failed: " + sessionError);
        if (session != null) {
            try {
                session.terminate();
            } catch (terminateError) {
            }
        }
        return false;
    }
}

trySession(".*CS_DAP.*");
trySession(".*SEC_AP.*");
trySession(".*CORTEX_M0P.*");

server.stop();
