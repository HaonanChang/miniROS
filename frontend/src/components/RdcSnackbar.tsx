import CancelIcon from "@mui/icons-material/Cancel";
import CheckCircleIcon from "@mui/icons-material/CheckCircle";
import CloseIcon from "@mui/icons-material/Close";
import InfoIcon from "@mui/icons-material/Info";
import WarningIcon from "@mui/icons-material/Warning";
import { SnackbarContent } from "@mui/material";
import IconButton from "@mui/material/IconButton";
import Snackbar from "@mui/material/Snackbar";
import { useDispatch, useSelector } from "react-redux";
import * as UUID from "uuid";
import { clearSnackbar, SnackbarType } from "../redux/SnackbarState";
import { RootState } from "../redux/Store";

export default function RdcSnackbar() {
	const dispatcher = useDispatch();

	const snackbarState = useSelector((state: RootState) => state.snackbar);

	function handleClose() {
		dispatcher(clearSnackbar());
	}

	let bgColor = undefined;

	switch (snackbarState.type) {
		case SnackbarType.ERROR:
			bgColor = "#b3261e";
			break;
		case SnackbarType.WARNING:
			bgColor = "#FFA500";
			break;
		case SnackbarType.INFO:
			bgColor = "#2196f3";
			break;
		case SnackbarType.GOOD:
			bgColor = "#137F0B";
			break;
		default:
			//bgColor = "bg-inherit";
			break;
	}

	const fgColor = bgColor !== undefined ? undefined : "text-white";

	const iconMap: Record<SnackbarType, React.ReactNode> = {
		[SnackbarType.ERROR]: (
			<CancelIcon fontSize="medium" sx={{ color: fgColor ?? "white" }} />
		),
		[SnackbarType.WARNING]: (
			<WarningIcon fontSize="medium" sx={{ color: fgColor ?? "white" }} />
		),
		[SnackbarType.INFO]: <InfoIcon fontSize="medium" />,
		[SnackbarType.GOOD]: (
			<CheckCircleIcon
				fontSize="medium"
				sx={{ color: fgColor ?? "white" }}
			/>
		),
		[SnackbarType.CLOSED]: <></>,
	};

	const content = (type: SnackbarType) => (
		<div className="flex flex-row items-center">
			{type === SnackbarType.CLOSED ? (
				<></>
			) : (
				<div className="mr-3">{iconMap[type]}</div>
			)}

			<div className="text-md opacity-85 text-wrap mr-15s">
				{snackbarState.message}
			</div>
		</div>
	);

	return (
		<Snackbar
			anchorOrigin={{ vertical: "bottom", horizontal: "center" }}
			open={snackbarState.type !== SnackbarType.CLOSED}
			autoHideDuration={snackbarState.durationMs}
			onClose={handleClose}
			key={UUID.v4().toString()}
		>
			<SnackbarContent
				sx={{
					backgroundColor: bgColor,
					color: fgColor,
					boxShadow: "none",
					borderRadius: "16px",
					padding: "12px",
					fontSize: "1.1rem",
				}}
				message={content(snackbarState.type)}
				action={[
					<IconButton
						key="close"
						aria-label="close"
						color="inherit"
						onClick={handleClose}
					>
						<CloseIcon fontSize="medium" />
					</IconButton>,
				]}
			></SnackbarContent>
		</Snackbar>
	);
}
