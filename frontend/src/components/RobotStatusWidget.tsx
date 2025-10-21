import CheckCircleIcon from "@mui/icons-material/CheckCircle";
import CloseIcon from "@mui/icons-material/Close";
import FiberManualRecordIcon from "@mui/icons-material/FiberManualRecord";
import { useEffect, useState } from "react";
import { useSelector } from "react-redux";
import { CommanderState } from "../redux/RobotInfo";
import { RootState } from "../redux/Store";
import "../styles/Tailwind.css";
import Globals from "../util/Globals";

function getStateDescription(state: CommanderState, isSaving: boolean) {
	switch (state) {
		case CommanderState.DEAD:
			return "We have no connection to the robot";
		case CommanderState.WAITING:
			return "We have connection to Commander! Waiting for robot to be ready...";
		case CommanderState.INIT:
			return "Robot is initializing. Please make sure you have GELLO put to home position.";
		case CommanderState.PREP_RECORD:
			return "CommanderState.PREP_RECORD"
		case CommanderState.AP_PRE_TELEOP:
			return "Aligning robot to GELLO...";
		case CommanderState.ACTIVE:
			return "CommanderState.ACTIVE";
		case CommanderState.AP_READY:
			// return `Robot is ${state.toLowerCase().replace("ap_", "")}! Use the left paddle to ${state === CommanderState.ACTIVE ? "start recording" : "start inference"} when you are ready.`;
			return `Robot is ${state.toLowerCase().replace("ap_", "")}! Use the left paddle to start when you are ready.`;
		case CommanderState.AP_INFERENCE:
			return "Robot is running inference...Use the Middle paddle to interrupt. Or right paddle to save if you think the data is good.";
		case CommanderState.AP_ALIGN:
			return "Robot stopped! Please move the GELLO to match the robot.";
		case CommanderState.AP_ALIGN_READY:
			return "GELLO is aligned to robot! Without moving the GELLOs, use the left paddle to take control.";
		case CommanderState.AP_TELEOP:
			return "Robot in teleop mode!";
		case CommanderState.RECORD:
			return "Robot is recording.";
		case CommanderState.RESTORE:
			return isSaving ? "Saving data and resetting!" : "Data discarded!";
		case CommanderState.DRAG:
			return "Drag the robot back to home position! Then use the right paddle to resume collection.";
		case CommanderState.REBOOT:
			return "Wait for robot to reboot!";
		default:
			return "Robot is in unknown / faulty state, please use the support button.";
	}
}

function getStateName(state: CommanderState) {
	if (state === CommanderState.DEAD) {
		return "Offline";
	}
	if (state === CommanderState.ACTIVE) {
		return "Ready";
	} else if (
		state === CommanderState.PREP_RECORD ||
		state === CommanderState.AP_PRE_TELEOP
	) {
		return "Aligning Robot...";
	}

	if (state === CommanderState.AP_ALIGN_READY) {
		return "Ready for Recovery";
	}

	const rawName = CommanderState[state].toLowerCase().replace("ap_", "");

	if (rawName.length > 0) {
		// Capitalize the first letter
		return rawName[0].toUpperCase() + rawName.slice(1);
	}

	return "Unknown";
}

export default function RobotStatusWidget() {
	const robotInfo = useSelector((state: RootState) => state.robotInfo);
	const [elapsedTimeMs, setElapsedTimeMs] = useState(0);
	//const [backgroundColor, setBackgroundColor] = useState("");

	let backgroundColor;

	// Logger.info("Commander state", robotInfo.commander.toString());

	// 02/19/2025: Per Jingru Zhao's feedback on "ACTIVE" state being misleading when GELLO is not in home position
	//const robotState = robotInfo.isGelloHome ? robotInfo.commander : CommanderState.INIT;
	// TODO: This is poorly designed, but it works for now

	let robotState = robotInfo.commander;

	if (
		robotInfo.commander === CommanderState.ACTIVE &&
		!robotInfo.isGelloHome
	) {
		robotState = CommanderState.INIT;
	} else if (
		robotInfo.commander === CommanderState.AP_ALIGN &&
		robotInfo.isGelloHome
	) {
		robotState = CommanderState.AP_ALIGN_READY;
	}

	// const robotState =
	// 	robotInfo.commander === CommanderState.ACTIVE && !robotInfo.isGelloHome
	// 		? CommanderState.INIT
	// 		: robotInfo.commander;

	// For the record timer
	useEffect(() => {
		const timerInterval = setInterval(() => {
			if (robotInfo.startRecordTimePosix && robotInfo.isRecording) {
				const now = new Date();
				const diff = now.getTime() - robotInfo.startRecordTimePosix;
				setElapsedTimeMs(diff);
			}
		}, 100);

		return () => {
			clearInterval(timerInterval);
		};
	}, [
		robotInfo.startRecordTimePosix,
		robotInfo.commander,
		robotInfo.isRecording,
	]);

	switch (robotState) {
		case CommanderState.DEAD:
		case CommanderState.INIT:
		case CommanderState.AP_ALIGN:
			backgroundColor = "grey";
			break;
		case CommanderState.ACTIVE:
		case CommanderState.AP_READY:
		case CommanderState.AP_ALIGN_READY:
		case CommanderState.PREP_RECORD:
		case CommanderState.AP_PRE_TELEOP:
			backgroundColor = "#2196f3";
			break;
		case CommanderState.WAITING:
			backgroundColor = "orange";
			break;
		case CommanderState.RECORD:
		case CommanderState.AP_INFERENCE:
		case CommanderState.AP_TELEOP:
			backgroundColor = "#FFA500";
			break;
		case CommanderState.RESTORE:
			backgroundColor = robotInfo.isDataSaving ? "#137F0B" : "#b3261e";
			break;
		case CommanderState.DRAG:
			backgroundColor = "#f30f03ff";
			break;
		case CommanderState.REBOOT:
			backgroundColor = "#f15f0aff";
			break;
		case CommanderState.FAULT:
			backgroundColor = "#b3261e";
			break;
	}

	const widthClass = "w-150";

	const discardWidget = (
		<div className="flex flex-row items-center justify-center w-full">
			<CloseIcon sx={{ fontSize: "48pt", color: "white" }} />
			<p className="text-5xl opacity-70 text-white ml-5">Discarded!</p>
		</div>
	);

	const saveWidget = (
		<div className="flex flex-row items-center justify-center w-full">
			<CheckCircleIcon sx={{ fontSize: "48pt", color: "white" }} />
			<p className="text-5xl opacity-70 text-white ml-5">Saved!</p>
		</div>
	);

	function isRecordWidgetVisible() {
		return (
			new Set([
				CommanderState.RECORD,
				CommanderState.AP_TELEOP,
				CommanderState.AP_INFERENCE,
			]).has(robotInfo.commander) &&
			robotInfo.startRecordTimePosix != undefined
		);
	}
	function getStopWatchTimeStr(ms: number) {
		const seconds = Math.floor(ms / 1000) % 60;
		const minutes = Math.floor(ms / 1000 / 60) % 60;
		const hours = Math.floor(ms / 1000 / 60 / 60);

		// pad with leading zeros
		return `${hours.toString().padStart(2, "0")}:${minutes.toString().padStart(2, "0")}:${seconds.toString().padStart(2, "0")}`;
	}

	return (
		<div
			className={`flex flex-col items-center justify-center gap-y-5 ${Globals.BG_SURFACE_CONTAINER} rounded-4xl transition-all`}
		>
			<div
				id="status_grid"
				className={`${widthClass} aspect-video rounded-4xl flex drop-shadow-lg transition-all`}
				style={{ backgroundColor: backgroundColor }}
			>
				{isRecordWidgetVisible() ? (
					<div className="flex flex-col items-center justify-center h-min mt-5 w-full">
						<div className="flex flex-row items-center gap-x-2 bg-white rounded-full p-1 shadow-lg shadow-black/10">
							<FiberManualRecordIcon
								sx={{
									fontSize: "30pt",

									// Blink the red dot every 600ms
									color:
										Math.floor(elapsedTimeMs / 600) % 2 ===
											0 || !robotInfo.isRecording
											? "red"
											: "transparent",
								}}
							/>
							<p className="text-md text-wrap font-mono mr-2 text-black/75">
								{getStopWatchTimeStr(elapsedTimeMs)}
							</p>
						</div>
					</div>
				) : null}

				{robotInfo.commander === CommanderState.RESTORE ? (
					robotInfo.isDataSaving ? (
						saveWidget
					) : (
						discardWidget
					)
				) : (
					<></>
				)}
			</div>
			<div
				className={`flex flex-col gap-y-1 items-center ${widthClass} px-10 transition-all pb-5`}
			>
				<div className="flex flex-row items-center justify-center">
					<p className="font-bold text-4xl fg-current">
						{getStateName(robotState)}
					</p>
				</div>

				<p className="text-md opacity-70 text-wrap">
					{getStateDescription(robotState, robotInfo.isDataSaving)}
				</p>
			</div>
		</div>
	);
}
