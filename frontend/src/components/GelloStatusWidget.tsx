import { useSelector } from "react-redux";
import { CommanderState, RobotInfo } from "../redux/RobotInfo";
import { RootState } from "../redux/Store";
import Globals from "../util/Globals";
import GelloPositionWidget from "./GelloPositionWidget";

/**
 * Returns true if the commander wishes to align with the robot rather than home position
 * @param robotInfo The robot info
 * @returns True if the commander wishes to align with the robot
 */
function isRobotAsHome(robotInfo: RobotInfo) {
	return new Set([
		CommanderState.AP_ALIGN,
		CommanderState.AP_ALIGN_READY,
		CommanderState.AP_PRE_TELEOP,
		CommanderState.AP_TELEOP,
	]).has(robotInfo.commander);
}

/**
 * Returns the text to display in the home widget (Inference should say "aligned", regular commander should say "at home")
 * @param robotInfo The robot info
 * @returns The text to display in the home widget
 */
function getHomeWidgetText(robotInfo: RobotInfo) {
	const keyText = isRobotAsHome(robotInfo) ? "aligned" : "at home";

	if (robotInfo.isGelloHome) {
		// Capitalize the first letter of the keyText
		return `${keyText.charAt(0).toUpperCase()}${keyText.slice(1)}`;
	} else {
		return `NOT ${keyText}`;
	}
}

export default function GelloAlignWidget() {
	const robotInfo = useSelector((state: RootState) => state.robotInfo);

	const homeWidgetColor = robotInfo.isGelloHome
		? "bg-green-500"
		: "bg-red-500";

	const isGelloOnline =
		Object.entries(robotInfo.gelloState["left"] ?? {}).length > 0 &&
		robotInfo.commander != CommanderState.DEAD;

	const statusText = isGelloOnline ? "Gello Online" : "Gello Offline";
	return (
		<div
			className={`flex flex-row items-center p-5 ${Globals.BG_SURFACE_CONTAINER} rounded-4xl transition-all gap-x-5`}
		>
			<div className="flex flex-col flex-3 transition-all gap-y-3">
				<p className="text-md font-bold">{statusText}</p>

				{isGelloOnline ? <GelloPositionWidget /> : null}

				{isGelloOnline ? (
					<div className="bg-current/10 mx-auto p-2 rounded-xl">
						<p className="text-sm opacity-60">
							Values indicate how far you are from the{" "}
							{isRobotAsHome(robotInfo) ? "ALIGNED" : "HOME"}{" "}
							position.
						</p>
					</div>
				) : null}
			</div>

			{isGelloOnline ? (
				<div className="transition-all">
					<div
						className={`flex justify-center items-center ${homeWidgetColor} text-white rounded-2xl transition-all`}
					>
						<span className="text-lg font-semibold p-5">
							{getHomeWidgetText(robotInfo)}
						</span>
					</div>
				</div>
			) : null}
		</div>
	);
}
