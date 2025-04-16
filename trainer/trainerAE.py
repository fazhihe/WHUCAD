import torch
import torch.optim as optim
from tqdm import tqdm
from model import CADTransformer
from .base import BaseTrainer
from .loss import CADLoss
from .scheduler import GradualWarmupScheduler
from cadlib.macro import *


class TrainerAE(BaseTrainer):
    def build_net(self, cfg):
        self.net = CADTransformer(cfg).cuda()

    def set_optimizer(self, cfg):
        """set optimizer and lr scheduler used in training"""
        self.optimizer = optim.Adam(self.net.parameters(), cfg.lr)
        self.scheduler = GradualWarmupScheduler(self.optimizer, 1.0, cfg.warmup_step)

    def set_loss_function(self):
        self.loss_func = CADLoss(self.cfg).cuda()

    def forward(self, data):
        commands = data['command'].cuda() # (N, S)
        args = data['args'].cuda()  # (N, S, N_ARGS)

        try:
            outputs = self.net(commands, args)
        except:
            print(data)
        loss_dict = self.loss_func(outputs)

        return outputs, loss_dict

    def encode(self, data, is_batch=False):
        """encode into latent vectors"""
        commands = data['command'].cuda()
        args = data['args'].cuda()
        if not is_batch:
            commands = commands.unsqueeze(0)
            args = args.unsqueeze(0)
        z = self.net(commands, args, encode_mode=True)
        return z

    def decode(self, z):
        """decode given latent vectors"""
        outputs = self.net(None, None, z=z, return_tgt=False)
        return outputs

    def logits2vec(self, outputs, refill_pad=True, to_numpy=True):
        """network outputs (logits) to final CAD vector"""
        out_command = torch.argmax(torch.softmax(outputs['command_logits'], dim=-1), dim=-1)  # (N, S)
        out_args = torch.argmax(torch.softmax(outputs['args_logits'], dim=-1), dim=-1) - 1  # (N, S, N_ARGS)
        if refill_pad: # fill all unused element to -1
            mask = ~torch.tensor(CMD_ARGS_MASK).bool().cuda()[out_command.long()]
            out_args[mask] = -1

        out_cad_vec = torch.cat([out_command.unsqueeze(-1), out_args], dim=-1)
        if to_numpy:
            out_cad_vec = out_cad_vec.detach().cpu().numpy()
        return out_cad_vec

    def evaluate(self, test_loader):
        """evaluatinon during training"""
        self.net.eval()
        pbar = tqdm(test_loader)
        pbar.set_description("EVALUATE[{}]".format(self.clock.epoch))

        all_ext_args_comp = []
        all_line_args_comp = []
        all_arc_args_comp = []
        all_circle_args_comp = []
        all_rev_args_comp = []
        all_spline_args_comp = []
        all_pocket_args_comp = []
        all_groove_args_comp = []
        all_shell_args_comp = []
        all_chamfer_args_comp = []
        all_fillet_args_comp = []
        all_draft_args_comp = []
        all_select_args_comp = []
        all_plane_args_comp = []
        all_hole_args_comp = []

        for i, data in enumerate(pbar):
            with torch.no_grad():
                commands = data['command'].cuda()
                args = data['args'].cuda()
                outputs = self.net(commands, args)
                out_args = torch.argmax(torch.softmax(outputs['args_logits'], dim=-1), dim=-1) - 1
                out_args = out_args.long().detach().cpu().numpy()  # (N, S, n_args)

            gt_commands = commands.squeeze(1).long().detach().cpu().numpy() # (N, S)
            gt_args = args.squeeze(1).long().detach().cpu().numpy() # (N, S, n_args)

            ext_pos = np.where(gt_commands == EXT_IDX)
            line_pos = np.where(gt_commands == LINE_IDX)
            arc_pos = np.where(gt_commands == ARC_IDX)
            circle_pos = np.where(gt_commands == CIRCLE_IDX)

            spline_pos = np.where(gt_commands == SCP_IDX)
            revolve_pos = np.where(gt_commands == REV_IDX)
            pocket_pos = np.where(gt_commands == POCKET_IDX)
            groove_pos = np.where(gt_commands == GROOVE_IDX)
            shell_pos = np.where(gt_commands == SHELL_IDX)
            chamfer_pos = np.where(gt_commands == CHAMFER_IDX)
            fillet_pos = np.where(gt_commands == FILLET_IDX)
            draft_pos = np.where(gt_commands == DRAFT_IDX)
            hole_pos = np.where(gt_commands == HOLE_IDX)
            select_pos = np.where(gt_commands == SELECT_IDX)
            plane_pos = np.where((gt_commands == EXT_IDX) | (gt_commands == REV_IDX) | (gt_commands == POCKET_IDX) | (gt_commands == GROOVE_IDX))


            args_comp = (gt_args == out_args).astype(np.int)
            all_ext_args_comp.append(args_comp[ext_pos][:, [12, 13, 14, 15, 18]])
            all_line_args_comp.append(args_comp[line_pos][:, :2])
            all_arc_args_comp.append(args_comp[arc_pos][:, :4])
            all_circle_args_comp.append(args_comp[circle_pos][:, [0, 1, 4]])

            all_rev_args_comp.append(args_comp[revolve_pos][:, [16, 17, 18]])
            all_spline_args_comp.append(args_comp[spline_pos][:, :2])
            all_pocket_args_comp.append(args_comp[pocket_pos][:, [12, 13, 14, 15]])
            all_groove_args_comp.append(args_comp[groove_pos][:, [16, 17]])
            all_shell_args_comp.append(args_comp[shell_pos][:, 19:21])
            all_chamfer_args_comp.append(args_comp[chamfer_pos][:, 21:23])
            all_fillet_args_comp.append(args_comp[fillet_pos][:, 23:24])
            all_draft_args_comp.append(args_comp[draft_pos][:, 24:25])
            all_hole_args_comp.append(args_comp[hole_pos][:, 25:28])
            all_select_args_comp.append(args_comp[select_pos][:, -N_ARGS_SELECT_PARAM:])
            all_plane_args_comp.append(args_comp[plane_pos][:, [5, 6, 7, 8, 9, 10, 11]])

        all_plane_args_comp = np.concatenate(all_plane_args_comp, axis=0)
        sket_plane_acc = np.mean(all_plane_args_comp[:, :N_ARGS_PLANE])
        sket_trans_acc = np.mean(all_plane_args_comp[:, N_ARGS_PLANE:N_ARGS_PLANE+N_ARGS_TRANS])

        extent_one_acc = np.mean(np.concatenate(all_ext_args_comp, axis=0))
        line_acc = np.mean(np.concatenate(all_line_args_comp, axis=0))
        arc_acc = np.mean(np.concatenate(all_arc_args_comp, axis=0))
        circle_acc = np.mean(np.concatenate(all_circle_args_comp, axis=0))

        rev_acc = np.mean(np.concatenate(all_rev_args_comp, axis=0))
        spline_acc = np.mean(np.concatenate(all_spline_args_comp, axis=0))
        pocket_acc = np.mean(np.concatenate(all_pocket_args_comp, axis=0))
        groove_acc = np.mean(np.concatenate(all_groove_args_comp, axis=0))
        shell_acc = np.mean(np.concatenate(all_shell_args_comp, axis=0))
        chamfer_acc = np.mean(np.concatenate(all_chamfer_args_comp, axis=0))
        fillet_acc = np.mean(np.concatenate(all_fillet_args_comp, axis=0))
        draft_acc = np.mean(np.concatenate(all_draft_args_comp, axis=0))
        hole_acc = np.mean(np.concatenate(all_hole_args_comp, axis=0))
        select_acc = np.mean(np.concatenate(all_select_args_comp, axis=0))

        self.val_tb.add_scalars("args_acc",
                                {"line": line_acc, "arc": arc_acc, "circle": circle_acc,
                                 "plane": sket_plane_acc, "trans": sket_trans_acc, "extent": extent_one_acc,
                                 "revolve": rev_acc, 'pocket': pocket_acc, 'groove': groove_acc,
                                 'spline': spline_acc, 'shell': shell_acc, 'chamfer': chamfer_acc,
                                 'fillet': fillet_acc, 'draft': draft_acc, 'hole': hole_acc,
                                 'select': select_acc},
                                global_step=self.clock.epoch)
