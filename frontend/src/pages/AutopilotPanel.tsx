import { WarningAmberRounded } from "@mui/icons-material";
import LockIcon from "@mui/icons-material/Lock";
import LockOpenIcon from "@mui/icons-material/LockOpen";
import { Button } from "@mui/material";
import { useDispatch, useSelector } from "react-redux";
import Counter from "../components/Counter";
import GelloAlignWidget from "../components/GelloStatusWidget";
import PromptSelector from "../components/PromptSelector";
import RobotStatusWidget from "../components/RobotStatusWidget";
import { setActionLock } from "../redux/ActionLock";
import { RootState } from "../redux/Store";
import Globals from "../util/Globals";

export default function AutopilotPanel() {
	const isLocked = useSelector(
		(state: RootState) => state.actionLock.actionLock,
	);
	const dispatch = useDispatch();
	return (
		<div className="flex flex-row gap-x-5 items-center">
			<div className="flex flex-col gap-y-5 flex-1">
				<RobotStatusWidget />
			</div>

			<div
				className={`flex flex-col flex-1 w-full ${Globals.BG_SURFACE_CONTAINER} rounded-2xl p-5 gap-5`}
			>
				<div className="flex flex-row items-center justify-between w-full gap-10">
					<Counter />

					<Button
						variant="contained"
						color="primary"
						sx={{
							borderRadius: "16px",
							textTransform: "none",
							backgroundColor: isLocked ? "red" : "green",
							color: "white",
						}}
						onClick={() => {
							dispatch(setActionLock(!isLocked));
						}}
					>
						{isLocked ? <LockIcon /> : <LockOpenIcon />}
						<p className="ml-2">
							{isLocked ? "Paddles Locked" : "Paddles Unlocked"}
						</p>
					</Button>
				</div>

				<div className="py-10 justify-center">
					<PromptSelector />
					<div className="flex flex-row mt-5 p-2 rounded-2xl gap-3 items-center justify-center bg-orange-500/25 outline outline-current/10">
						<WarningAmberRounded className="text-orange-500" />
						<p>Please lock paddles before updating prompts!</p>
					</div>
				</div>

				<GelloAlignWidget />
			</div>
		</div>
	);
}
