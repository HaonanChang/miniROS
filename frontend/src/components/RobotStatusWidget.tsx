import CheckCircleIcon from "@mui/icons-material/CheckCircle";
import CloseIcon from "@mui/icons-material/Close";
import FiberManualRecordIcon from "@mui/icons-material/FiberManualRecord";
import { useEffect, useState } from "react";
import { useTranslation } from "react-i18next";
import { useSelector } from "react-redux";
import Globals from "../misc/Globals";
import { CommanderState } from "../redux/RobotInfo";
import { RootState } from "../redux/Store";
import "../styles/Tailwind.css";

export default function RobotStatusWidget() {
	const { t } = useTranslation();
	const robotInfo = useSelector((state: RootState) => state.robotInfo);
	const [elapsedTimeMs, setElapsedTimeMs] = useState(0);
	//const [backgroundColor, setBackgroundColor] = useState("");

	function getStateDescription(state: CommanderState, isSaving: boolean) {
		return t(
			"cmd_state/subtitle/" +
				state.toLowerCase() +
				(isSaving && state === CommanderState.RESTORE ? "_saving" : ""),
		);
	}

	function getStateName(state: CommanderState) {
		return t("cmd_state/title/" + state.toLowerCase());
	}

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
		case CommanderState.STOPPED:
			backgroundColor = "#24231f";
			break;
	}

	const widthClass = "w-150";

	const discardWidget = (
		<div className="flex flex-row items-center justify-center w-full">
			<CloseIcon sx={{ fontSize: "48pt", color: "white" }} />
			<p className="text-5xl opacity-70 text-white ml-5">
				{t("cmd_state/misc/episode_discarded")}
			</p>
		</div>
	);

	const saveWidget = (
		<div className="flex flex-row items-center justify-center w-full">
			<CheckCircleIcon sx={{ fontSize: "48pt", color: "white" }} />
			<p className="text-5xl opacity-70 text-white ml-5">
				{t("cmd_state/misc/episode_saved")}
			</p>
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
