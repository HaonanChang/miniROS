import RestartAltIcon from "@mui/icons-material/RestartAlt";
import { Button } from "@mui/material";
import { useDispatch, useSelector } from "react-redux";
import RobotRequest from "../backend/RobotRequest";
import { setSnackBar, SnackbarType } from "../redux/SnackbarState";
import { RootState } from "../redux/Store";
import Globals from "../util/Globals";

export default function Counter() {
	const value = useSelector((state: RootState) => state.robotInfo.counter);
	const dispatch = useDispatch();

	const handleReset = async () => {
		try {
			await RobotRequest.resetCounter();

			dispatch(
				setSnackBar({
					message: "Counter reset!",
					type: SnackbarType.GOOD,
					durationMs: 1500,
				}),
			);
		} catch (e: unknown) {
			dispatch(
				setSnackBar({
					message: "Failed to reset counter! " + (e as Error).message,
					type: SnackbarType.ERROR,
					durationMs: 1500,
				}),
			);
		}
	};

	return (
		<div
			className={`flex flex-row rounded-2xl ${Globals.BG_SURFACE_CONTAINER} h-min items-center justify-center`}
		>
			<div className="px-5">
				<p className="text-lg text-black dark:text-white font-bold">
					Count: {value}
				</p>
			</div>
			<Button
				variant="contained"
				color="error"
				sx={{
					borderRadius: "16px",
					textTransform: "none",
				}}
				disabled={value === 0}
				onClick={handleReset}
			>
				<div className="flex flex-row items-center justify-center gap-x-1">
					<RestartAltIcon />
					<p className="text-lg">Reset</p>
				</div>
			</Button>
		</div>
	);
}
