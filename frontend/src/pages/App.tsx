import { useMediaQuery } from "@mui/material";
import { createTheme, ThemeProvider } from "@mui/material/styles";
import { useEffect, useMemo } from "react";
import { useDispatch, useSelector } from "react-redux";
import KeyListener from "../backend/KeyboardListener";
import RobotRequest from "../backend/RobotRequest";
import RobotStateListener from "../backend/RobotStateListener";
import RdcSnackbar from "../components/RdcSnackbar";
import { RootState } from "../redux/Store";
import "../styles/App.css";
import "../styles/Tailwind.css";
import AutopilotPanel from "./AutopilotPanel";
import NormalPanel from "./NormalPanel";

function App() {
	const dispatcher = useDispatch();

	const prefersDarkMode = useMediaQuery("(prefers-color-scheme: dark)");

	const theme = useMemo(
		() =>
			createTheme({
				palette: {
					mode: prefersDarkMode ? "dark" : "light",
				},
			}),
		[prefersDarkMode],
	);

	// const robotInfo = useSelector((state: RootState) => state.robotInfo);

	useEffect(() => {
		RobotStateListener.start(dispatcher);
		KeyListener.init();

		RobotRequest.setDispatcher(dispatcher);

		const triggers = {
			s: () => RobotRequest.safeAction(RobotRequest.leftPaddle),
			d: () => RobotRequest.safeAction(RobotRequest.middlePaddle),
			q: () => RobotRequest.safeAction(RobotRequest.rightPaddle),
		};

		for (const [key, trigger] of Object.entries(triggers)) {
			KeyListener.getInstance().removeKeyPressListener(key, trigger);
			KeyListener.getInstance().registerKeyPressListener(key, trigger);
		}

		// On lifecycle ends, do this
		return () => {
			(() => RobotStateListener.stop())();
		};
	}, []);

	// const currentState = useSelector(
	// 	(state: RootState) => state.robotInfo.state,
	// );
	const isAutopilot = useSelector(
		(state: RootState) => state.robotInfo.isAutopilot,
	);

	return (
		<>
			<ThemeProvider theme={theme}>
				{isAutopilot ? <AutopilotPanel /> : <NormalPanel />}
				<RdcSnackbar />
			</ThemeProvider>
		</>
	);
}

export default App;
